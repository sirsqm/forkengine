// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/renderer3d/Renderer3D.h"

#include "fe/anim/SkinnedMesh.h"
#include "fe/anim/SkinnedMeshRenderer3D.h"
#include "fe/core/Assert.h"
#include "fe/core/Frustum.h"
#include "fe/core/Log.h"
#include "fe/resources/ResourceManager.h"
#include "fe/resources/Texture2D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/scene3d/DirectionalLight3D.h"
#include "fe/scene3d/MeshRenderer3D.h"
#include "fe/scene3d/PointLight3D.h"
#include "fe/scene3d/SpotLight3D.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

namespace fe::renderer3d {
namespace {

constexpr const char* kVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;

// The per-instance stream (VertexLayout::mesh_instance): the instance's model
// matrix and its normal matrix, each as four column vectors. One draw call
// covers every instance of a mesh, so the transform travels with the instance
// rather than in a uniform.
layout(location = 3) in vec4 a_model_0;
layout(location = 4) in vec4 a_model_1;
layout(location = 5) in vec4 a_model_2;
layout(location = 6) in vec4 a_model_3;
layout(location = 7) in vec4 a_normal_0;
layout(location = 8) in vec4 a_normal_1;
layout(location = 9) in vec4 a_normal_2;

uniform mat4 u_view_projection;

out vec3 v_world_position;
out vec3 v_normal;
out vec2 v_uv;

void main() {
    // Constructed column-major, matching how the columns were uploaded.
    mat4 model = mat4(a_model_0, a_model_1, a_model_2, a_model_3);
    mat3 normal_matrix = mat3(a_normal_0.xyz, a_normal_1.xyz, a_normal_2.xyz);
    vec4 world = model * vec4(a_position, 1.0);
    v_world_position = world.xyz;
    v_normal = normal_matrix * a_normal;
    v_uv = a_uv;
    gl_Position = u_view_projection * world;
}
)glsl";

// The fragment shader's uniform array and its two-tile atlas layout are written
// for exactly this many cascades: the sampler is a plain 2D texture (a sampler
// array does not index dynamically in GLSL ES 3.00), so the count is baked into
// the tile offset and the `[2]` declaration. Raising the constant means editing
// both.
static_assert(kShadowCascades == 2, "the shadow shader is written for two cascades");

constexpr const char* kFragmentShaderSource = R"glsl(
#version 330 core

in vec3 v_world_position;
in vec3 v_normal;
in vec2 v_uv;

uniform sampler2D u_texture;
uniform vec4 u_base_color;
uniform float u_specular;
// Blinn-Phong exponent. Derived on the CPU from the material's roughness, so the
// shader stays a single lobe while the material stays metallic-roughness.
uniform float u_shininess;
// Emissive colour, added after lighting and not affected by any light.
uniform vec3 u_emissive;
uniform sampler2D u_emissive_map;
uniform float u_emissive_map_enabled;

uniform vec3 u_camera_position;

// Directional light: global, no position.
uniform vec3 u_light_direction;  // Direction the light travels.
uniform vec3 u_light_color;
uniform float u_light_intensity;
uniform float u_ambient;

// Point lights. Each element packs position.xyz and range in .w.
uniform int u_point_light_count;
uniform vec4 u_point_light_position_range[8];
uniform vec4 u_point_light_color_intensity[8];  // rgb + intensity.

// Spot lights. Each light uses one vec4 in each of three arrays.
//   u_spot_light_data[i]        = position.xyz, range in .w
//   u_spot_light_data[i + 8]    = direction.xyz (the direction the light travels)
//   u_spot_light_color_intensity[i] = color.rgb, intensity in .w
//   u_spot_light_cone[i]        = cos(inner half-angle), cos(outer half-angle), 0, 0
uniform int u_spot_light_count;
uniform vec4 u_spot_light_data[16];
uniform vec4 u_spot_light_color_intensity[8];
uniform vec4 u_spot_light_cone[8];

out vec4 frag_color;

// Shadow map: depth rendered from the directional light's point of view, once
// per cascade, with the cascades sharing one atlas side by side. The comparison
// is done by hand (compare mode NONE) so the shader stays backend-agnostic;
// `u_light_space_matrix` maps world space into a cascade's clip space, and its
// .xyz/.w gives the depth the light saw at this fragment.
uniform sampler2D u_shadow_map;
uniform mat4 u_light_space_matrix[2];
uniform float u_shadow_bias;
// The view depth where the far cascade takes over, and the axis that depth is
// measured along. Both come from the same fit that produced the matrices, so a
// fragment is always sampled from the cascade that contains it.
uniform vec3 u_camera_forward;
uniform float u_cascade_split;

// 1.0 where the fragment is lit, 0.0 where it is occluded. Samples a 3x3
// neighbourhood and averages the pass/fail, which softens the hard texel edge
// enough to read as a shadow rather than a staircase.
float shadow_factor(vec3 world_position, vec3 to_light) {
    // Cascade selection by view depth: the near band is sampled from the cascade
    // fitted to it, which covers less of the world per texel than the far one.
    float depth = dot(world_position - u_camera_position, u_camera_forward);
    int cascade = depth <= u_cascade_split ? 0 : 1;

    vec4 light_space = u_light_space_matrix[cascade] * vec4(world_position, 1.0);
    vec3 projected = light_space.xyz / light_space.w;
    projected = projected * 0.5 + 0.5; // Clip [-1, 1] to texture [0, 1].

    // Outside the cascade there is no occlusion information, so treat it as lit.
    if (projected.x < 0.0 || projected.x > 1.0 || projected.y < 0.0 || projected.y > 1.0 || projected.z < 0.0 ||
        projected.z > 1.0) {
        return 1.0;
    }

    // The cascade's tile: cascade 0 is the left half of the atlas, cascade 1 the
    // right. The tile bounds below keep a 3x3 tap at the seam from reading the
    // other cascade's depths.
    vec2 tile_min = vec2(float(cascade) * 0.5, 0.0);
    vec2 tile_max = tile_min + vec2(0.5, 1.0);
    vec2 uv = vec2(projected.x * 0.5 + tile_min.x, projected.y);

    // Slope-scaled bias: a surface nearly edge-on to the light needs a larger
    // offset to avoid self-shadowing acne. The slope is how edge-on the surface
    // is to the light, so it is the surface normal against the light direction,
    // not a fixed world axis - a constant axis would bias a floor and a wall
    // differently for no reason.
    float slope = max(1.0 - dot(normalize(v_normal), to_light), 0.0);
    float bias = u_shadow_bias * (0.5 + slope);

    float occluded = 0.0;
    vec2 texel = 1.0 / vec2(textureSize(u_shadow_map, 0));
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            vec2 sample_uv = clamp(uv + vec2(float(x), float(y)) * texel, tile_min + texel, tile_max - texel);
            float closest = texture(u_shadow_map, sample_uv).r;
            occluded += projected.z - bias > closest ? 1.0 : 0.0;
        }
    }
    return 1.0 - occluded / 9.0;
}

// Blinn-Phong diffuse + specular for a light arriving along `to_light`
// (unit vector towards the light) with the given colour and distance falloff.
vec3 shade(vec3 n, vec3 v, vec3 to_light, vec3 light_color, float attenuation) {
    float diffuse = max(dot(n, to_light), 0.0);
    vec3 h = normalize(to_light + v);
    float spec = pow(max(dot(n, h), 0.0), u_shininess) * u_specular;
    return light_color * (diffuse + spec) * attenuation;
}

void main() {
    vec4 albedo = texture(u_texture, v_uv) * u_base_color;
    vec3 emissive = u_emissive * mix(vec3(1.0), texture(u_emissive_map, v_uv).rgb, u_emissive_map_enabled);

    vec3 n = normalize(v_normal);
    vec3 v = normalize(u_camera_position - v_world_position);

    // Ambient plus the directional light. Only the directional light casts a
    // shadow; ambient stays unshadowed so occluded areas keep some fill.
    vec3 l = normalize(-u_light_direction);
    float shadow = shadow_factor(v_world_position, l);
    vec3 lighting = vec3(u_ambient) + shadow * shade(n, v, l, u_light_color * u_light_intensity, 1.0);

    // Point lights, with an inverse-square falloff windowed to reach zero at
    // the light's range so a light has a bounded influence.
    for (int i = 0; i < u_point_light_count && i < 8; ++i) {
        vec3 to_light = u_point_light_position_range[i].xyz - v_world_position;
        float distance = length(to_light);
        float range = u_point_light_position_range[i].w;
        if (range <= 0.0) {
            continue;
        }
        float window = max(1.0 - distance / range, 0.0);
        float attenuation = window * window;
        lighting += shade(n, v, to_light / max(distance, 1e-4), u_point_light_color_intensity[i].rgb *
                          u_point_light_color_intensity[i].a, attenuation);
    }

    // Spot lights: a point light gated by a smooth cone.
    for (int i = 0; i < u_spot_light_count && i < 8; ++i) {
        vec3 to_light = u_spot_light_data[i].xyz - v_world_position;
        float distance = length(to_light);
        float range = u_spot_light_data[i].w;
        if (range <= 0.0) {
            continue;
        }
        vec3 light_dir = u_spot_light_data[i + 8].xyz;   // Direction the light travels.
        float cos_inner = u_spot_light_cone[i].x;
        float cos_outer = u_spot_light_cone[i].y;

        float cos_angle = dot(normalize(-to_light), light_dir);
        // Smoothstep between the outer and inner cosine gives a soft edge. The
        // division is guarded: the renderer keeps outer < inner, but a hand-set
        // equal pair must not divide by zero.
        float denom = max(cos_inner - cos_outer, 1e-4);
        float cone = clamp((cos_angle - cos_outer) / denom, 0.0, 1.0);

        float window = max(1.0 - distance / range, 0.0);
        float attenuation = window * window * cone;
        lighting += shade(n, v, to_light / max(distance, 1e-4), u_spot_light_color_intensity[i].rgb *
                          u_spot_light_color_intensity[i].a, attenuation);
    }

    frag_color = vec4(albedo.rgb * lighting + emissive, albedo.a);
}
)glsl";

// Depth-only pass. Renders the scene from the light's point of view; the depth
// buffer is the only output the shadow map needs. There is no colour work and
// no fragment output, so the driver can discard the fragment stage entirely.
//
// It reads the same per-instance stream the main pass does, so the shadow map
// is drawn with the same batching: one call per mesh, all its casters at once.
constexpr const char* kShadowVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 3) in vec4 a_model_0;
layout(location = 4) in vec4 a_model_1;
layout(location = 5) in vec4 a_model_2;
layout(location = 6) in vec4 a_model_3;

uniform mat4 u_light_view_projection;

void main() {
    mat4 model = mat4(a_model_0, a_model_1, a_model_2, a_model_3);
    gl_Position = u_light_view_projection * model * vec4(a_position, 1.0);
}
)glsl";

constexpr const char* kShadowFragmentShaderSource = R"glsl(
#version 330 core

void main() {
}
)glsl";

// The skinned lit vertex shader. It shares `kFragmentShaderSource` with the
// static path -- a skinned character and a static prop are lit by one material
// model, so the fragment stage is byte-identical and only the vertex stage
// differs.
//
// What differs, and why it is a second shader rather than a branch in the first:
//
//   * The vertex carries four joint indices and four weights (attributes 3 and
//     4). The indices arrive as floats because `VertexFormat` has no integer
//     kind; the shader converts, which is exact for any rig short of 2^24 joints.
//   * The palette is a `mat4` array uniform indexed by those joint indices, not
//     a per-instance attribute. A forty-joint rig would need forty attribute
//     slots, past every backend's limit; a uniform array is bounded by the
//     uniform budget instead. See `VertexLayout::skinned_instance`.
//   * The model matrix arrives in the per-instance stream at locations 5-11 (the
//     static path's 3-9 are taken by the joint attributes), so the two instance
//     layouts do not collide on a shared attribute slot.
//
// The skin is the standard linear blend: transform the vertex by each influencing
// joint's palette matrix, weight the results, and sum. The weights are normalised
// in the shader as well as on the CPU, so an asset whose weights do not sum to
// one renders at the right size rather than shrinking towards the origin, and a
// vertex with no usable influence falls back to its bind position.
constexpr const char* kSkinnedVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;
layout(location = 3) in vec4 a_joints;   // joint indices, as floats
layout(location = 4) in vec4 a_weights;

layout(location = 5) in vec4 a_model_0;
layout(location = 6) in vec4 a_model_1;
layout(location = 7) in vec4 a_model_2;
layout(location = 8) in vec4 a_model_3;
layout(location = 9) in vec4 a_normal_0;
layout(location = 10) in vec4 a_normal_1;
layout(location = 11) in vec4 a_normal_2;

// The skinning palette: world * inverse_bind per joint, uploaded once per
// instance. The cap matches Renderer3D::kMaxPaletteJoints; a rig larger than the
// cap is a shader change (chunk the palette across draws), not a data-model one.
uniform mat4 u_palette[128];

uniform mat4 u_view_projection;

out vec3 v_world_position;
out vec3 v_normal;
out vec2 v_uv;

void main() {
    mat4 model = mat4(a_model_0, a_model_1, a_model_2, a_model_3);
    mat3 normal_matrix = mat3(a_normal_0.xyz, a_normal_1.xyz, a_normal_2.xyz);

    // Sum the weights so a vertex whose authored weights do not sum to one is
    // normalised rather than scaled. A zero sum means no usable influence: the
    // vertex stays where the bind pose put it.
    float weight_sum = a_weights.x + a_weights.y + a_weights.z + a_weights.w;
    vec4 skin_position = vec4(0.0);
    vec3 skin_normal = vec3(0.0);
    if (weight_sum > 0.0) {
        ivec4 joints = ivec4(a_joints + 0.5); // round-to-nearest; the values are exact integers
        for (int i = 0; i < 4; ++i) {
            float weight = a_weights[i] / weight_sum;
            if (weight == 0.0) {
                continue;
            }
            mat4 skin = u_palette[joints[i]];
            skin_position += weight * (skin * vec4(a_position, 1.0));
            skin_normal += weight * (mat3(skin) * a_normal);
        }
    } else {
        skin_position = vec4(a_position, 1.0);
        skin_normal = a_normal;
    }

    vec4 world = model * skin_position;
    v_world_position = world.xyz;
    v_normal = normal_matrix * skin_normal;
    v_uv = a_uv;
    gl_Position = u_view_projection * world;
}
)glsl";

// The skinned depth-only vertex shader, for the shadow pass. It applies the same
// linear blend skinning as the lit pass so a character casts the shadow of its
// pose rather than of its bind shape.
constexpr const char* kSkinnedShadowVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 3) in vec4 a_joints;
layout(location = 4) in vec4 a_weights;

layout(location = 5) in vec4 a_model_0;
layout(location = 6) in vec4 a_model_1;
layout(location = 7) in vec4 a_model_2;
layout(location = 8) in vec4 a_model_3;

uniform mat4 u_palette[128];
uniform mat4 u_light_view_projection;

void main() {
    mat4 model = mat4(a_model_0, a_model_1, a_model_2, a_model_3);
    float weight_sum = a_weights.x + a_weights.y + a_weights.z + a_weights.w;
    vec4 skin_position = vec4(0.0);
    if (weight_sum > 0.0) {
        ivec4 joints = ivec4(a_joints + 0.5);
        for (int i = 0; i < 4; ++i) {
            float weight = a_weights[i] / weight_sum;
            if (weight == 0.0) {
                continue;
            }
            skin_position += weight * (u_palette[joints[i]] * vec4(a_position, 1.0));
        }
    } else {
        skin_position = vec4(a_position, 1.0);
    }
    gl_Position = u_light_view_projection * model * skin_position;
}
)glsl";

// Particle billboards. The vertex stage takes a shared unit quad (location 0) and
// one instance record per particle, and expands the quad into a world-space
// square that faces the camera: the quad's corner offsets are scaled by the
// particle's diameter and placed along the camera's right and up axes, so no
// per-particle rotation is stored and a billboard turns with the view for free.
// The fragment stage turns the square into a soft circle, because a hard-edged
// quad reads as a flat square no matter how it is blended.
constexpr const char* kParticleVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec2 a_corner;
layout(location = 1) in vec4 a_position_size;
layout(location = 2) in vec4 a_color;

uniform mat4 u_view_projection;
uniform vec3 u_camera_right;
uniform vec3 u_camera_up;

out vec2 v_uv;
out vec4 v_color;

void main() {
    vec3 world = a_position_size.xyz
               + u_camera_right * (a_corner.x * a_position_size.w)
               + u_camera_up * (a_corner.y * a_position_size.w);
    v_uv = a_corner;
    v_color = a_color;
    gl_Position = u_view_projection * vec4(world, 1.0);
}
)glsl";

constexpr const char* kParticleFragmentShaderSource = R"glsl(
#version 330 core

in vec2 v_uv;
in vec4 v_color;

out vec4 frag_color;

void main() {
    // v_uv is the corner in [-0.5, 0.5]; the radial distance is 0 at the centre
    // and 1 at the circle inscribed in the quad. Squaring the falloff keeps the
    // core solid and the rim soft, which is what reads as a particle rather than
    // a flat disc.
    float radius = length(v_uv) * 2.0;
    float falloff = clamp(1.0 - radius, 0.0, 1.0);
    float alpha = v_color.a * falloff * falloff;
    if (alpha <= 0.002) {
        discard;
    }
    frag_color = vec4(v_color.rgb, alpha);
}
)glsl";

// The shared per-vertex corners: a unit quad in [-0.5, 0.5], expanded per
// instance. Two triangles, counter-clockwise; the pass never culls, so a
// particle seen from behind is drawn too.
constexpr std::array<f32, 8> kParticleQuadVertices = {-0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f};

constexpr std::array<u32, 6> kParticleQuadIndices = {0, 1, 2, 0, 2, 3};

// The environment background: a full-screen quad whose fragment colour is a
// vertical gradient from `u_bottom_color` at the bottom of the frame to
// `u_top_color` at the top. The quad's position is already in clip space, so no
// camera matrix is needed and the gradient follows the screen rather than the
// camera's pitch, which is the cheap, always-correct behaviour for a first cut.
constexpr const char* kBackgroundVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_uv;

out vec2 v_uv;

void main() {
    v_uv = a_uv;
    gl_Position = vec4(a_position, 0.0, 1.0);
}
)glsl";

constexpr const char* kBackgroundFragmentShaderSource = R"glsl(
#version 330 core

in vec2 v_uv;

uniform vec3 u_top_color;
uniform vec3 u_bottom_color;

out vec4 frag_color;

void main() {
    frag_color = vec4(mix(u_bottom_color, u_top_color, v_uv.y), 1.0);
}
)glsl";

// Two triangles covering clip space, UV (0,0) at the bottom-left so the gradient
// runs bottom to top.
constexpr std::array<f32, 16> kBackgroundQuadVertices = {
    // position.xy   uv.xy
    -1.0f, -1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 1.0f,
};

constexpr std::array<u32, 6> kBackgroundQuadIndices = {0, 1, 2, 0, 2, 3};

// The normal matrix is transpose(inverse(upper-left 3x3 of model)), so a
// non-uniform scale does not skew normals. Returned in column-major order,
// which is what the RHI uploads.
[[nodiscard]] std::array<f32, 9> normal_matrix(const Mat4& model) noexcept {
    const Mat4 inv = inverse(model);
    std::array<f32, 9> out{};
    for (usize col = 0; col < 3; ++col) {
        for (usize row = 0; row < 3; ++row) {
            // transpose(inv).at(row, col) == inv.at(col, row)
            out[col * 3 + row] = inv.at(col, row);
        }
    }
    return out;
}

// The world-space AABB of a local AABB under `model` now lives in the shared
// extraction seam (`fe/render/RenderExtract.h`), because extraction bakes it
// into each `render::MeshInstance` and the culling pass is a pure function of
// that array. See ADR-0052.

} // namespace

Renderer3D::Renderer3D() = default;
Renderer3D::~Renderer3D() = default;

Error Renderer3D::initialize(rhi::Device& device) {
    rhi::ShaderDesc desc;
    desc.name = "renderer3d_lit";
    desc.vertex_source = kVertexShaderSource;
    desc.fragment_source = kFragmentShaderSource;

    auto shader = device.create_shader(desc);
    if (!shader) {
        return shader.error();
    }
    shader_ = shader.value();

    rhi::ShaderDesc shadow_desc;
    shadow_desc.name = "renderer3d_shadow";
    shadow_desc.vertex_source = kShadowVertexShaderSource;
    shadow_desc.fragment_source = kShadowFragmentShaderSource;
    auto shadow_shader = device.create_shader(shadow_desc);
    if (!shadow_shader) {
        return shadow_shader.error();
    }
    shadow_shader_ = shadow_shader.value();

    rhi::ShaderDesc skinned_desc;
    skinned_desc.name = "renderer3d_skinned";
    skinned_desc.vertex_source = kSkinnedVertexShaderSource;
    skinned_desc.fragment_source = kFragmentShaderSource;
    auto skinned_shader = device.create_shader(skinned_desc);
    if (!skinned_shader) {
        return skinned_shader.error();
    }
    skinned_shader_ = skinned_shader.value();

    rhi::ShaderDesc skinned_shadow_desc;
    skinned_shadow_desc.name = "renderer3d_skinned_shadow";
    skinned_shadow_desc.vertex_source = kSkinnedShadowVertexShaderSource;
    skinned_shadow_desc.fragment_source = kShadowFragmentShaderSource;
    auto skinned_shadow_shader = device.create_shader(skinned_shadow_desc);
    if (!skinned_shadow_shader) {
        return skinned_shadow_shader.error();
    }
    skinned_shadow_shader_ = skinned_shadow_shader.value();

    const u8 white[4] = {255, 255, 255, 255};
    rhi::TextureDesc white_desc;
    white_desc.name = "renderer3d_white";
    white_desc.width = 1;
    white_desc.height = 1;
    white_desc.pixels = white;
    auto white_texture = device.create_texture(white_desc);
    if (!white_texture) {
        return white_texture.error();
    }
    default_texture_ = white_texture.value();

    return Error::none();
}

const Camera3D* Renderer3D::find_active_camera(const Scene& scene) {
    const Camera3D* fallback = nullptr;
    const Camera3D* found = nullptr;
    scene.for_each_node([&](Node& node) {
        if (const auto* camera = node.get_component<Camera3D>(); camera != nullptr) {
            if (fallback == nullptr) {
                fallback = camera;
            }
            if (camera->primary && found == nullptr) {
                found = camera;
            }
        }
    });
    return found != nullptr ? found : fallback;
}

Result<const Renderer3D::GpuMesh*> Renderer3D::acquire_mesh(rhi::Device& device, const Mesh& mesh) {
    const auto it = buffers_.find(&mesh);
    if (it != buffers_.end()) {
        return &it->second;
    }

    if (mesh.vertex_count() == 0 || mesh.index_count() == 0) {
        return Error("renderer3d: mesh has no geometry");
    }

    auto vertex_buffer = device.create_vertex_buffer(rhi::VertexLayout::position_normal_uv(),
                                                     static_cast<u32>(mesh.vertex_count() * sizeof(Vertex3D)));
    if (!vertex_buffer) {
        return vertex_buffer.error();
    }
    auto index_buffer = device.create_index_buffer(static_cast<u32>(mesh.index_count()));
    if (!index_buffer) {
        return index_buffer.error();
    }

    GpuMesh gpu{};
    gpu.vertex_buffer = vertex_buffer.value();
    gpu.index_buffer = index_buffer.value();
    gpu.index_count = static_cast<u32>(mesh.index_count());

    gpu.vertex_buffer->update(std::span<const u8>(reinterpret_cast<const u8*>(mesh.vertices().data()),
                                                  mesh.vertices().size() * sizeof(Vertex3D)));
    gpu.index_buffer->update(
        std::span<const u8>(reinterpret_cast<const u8*>(mesh.indices().data()), mesh.indices().size() * sizeof(u32)));

    const auto [inserted, _] = buffers_.emplace(&mesh, std::move(gpu));
    return &inserted->second;
}

Result<const Renderer3D::GpuMesh*> Renderer3D::acquire_skinned_mesh(rhi::Device& device,
                                                                    const anim::SkinnedMesh& mesh) {
    const auto it = skinned_buffers_.find(&mesh);
    if (it != skinned_buffers_.end()) {
        return &it->second;
    }

    if (mesh.vertex_count() == 0 || mesh.index_count() == 0) {
        return Error("renderer3d: skinned mesh has no geometry");
    }

    // The skinned layout: the same position/normal/uv triple at locations 0-2,
    // then the joint indices and weights at 3 and 4. The stride is
    // sizeof(SkinnedVertex3D), which the layout's static_assert pins.
    auto vertex_buffer = device.create_vertex_buffer(rhi::VertexLayout::skinned_position_normal_uv(),
                                                     static_cast<u32>(mesh.vertex_count() * sizeof(SkinnedVertex3D)));
    if (!vertex_buffer) {
        return vertex_buffer.error();
    }
    auto index_buffer = device.create_index_buffer(static_cast<u32>(mesh.index_count()));
    if (!index_buffer) {
        return index_buffer.error();
    }

    GpuMesh gpu{};
    gpu.vertex_buffer = vertex_buffer.value();
    gpu.index_buffer = index_buffer.value();
    gpu.index_count = static_cast<u32>(mesh.index_count());

    gpu.vertex_buffer->update(std::span<const u8>(reinterpret_cast<const u8*>(mesh.vertices().data()),
                                                  mesh.vertices().size() * sizeof(SkinnedVertex3D)));
    gpu.index_buffer->update(
        std::span<const u8>(reinterpret_cast<const u8*>(mesh.indices().data()), mesh.indices().size() * sizeof(u32)));

    const auto [inserted, _] = skinned_buffers_.emplace(&mesh, std::move(gpu));
    return &inserted->second;
}

void Renderer3D::report_oversized_palette(const anim::SkinnedMesh& mesh, usize joints) {
    if (warned_large_rigs_.insert(&mesh).second) {
        FE_LOG_WARN("renderer3d: skinned mesh has %zu joints, more than the shader's %u; drawing the first %u", joints,
                    kMaxPaletteJoints, kMaxPaletteJoints);
    }
}

void Renderer3D::ensure_skinned_instance_capacity(rhi::Device& device, u32 count) {
    // Same rule as the static path: the record must match the layout's stride, or
    // every instance after the first reads its transform shifted. The skinned
    // record is the same `Instance` struct, so the size check is shared; only the
    // buffer's layout differs (attribute locations 5-11 rather than 3-9).
    static_assert(sizeof(Instance) == 7 * 16, "Instance must match VertexLayout::skinned_instance()'s stride");

    if (count <= skinned_instance_capacity_) {
        return;
    }
    const u32 new_capacity = std::max(count, skinned_instance_capacity_ * 2);
    auto buffer = device.create_vertex_buffer(rhi::VertexLayout::skinned_instance(),
                                              new_capacity * static_cast<u32>(sizeof(Instance)));
    if (!buffer) {
        FE_LOG_ERROR("renderer3d: could not grow skinned instance buffer: %s", buffer.error().message().c_str());
        return;
    }
    skinned_instance_buffer_ = buffer.value();
    skinned_instance_capacity_ = new_capacity;
}

void Renderer3D::build_skinned_instance_stream() {
    std::vector<Instance>& records = frame_skinned_instance_data_;
    records.clear();
    records.reserve(frame_skinned_instances_.size());
    for (const render::SkinnedInstance& instance : frame_skinned_instances_) {
        Instance record{};
        for (usize col = 0; col < 4; ++col) {
            record.model[col] = Vec4{instance.model.at(0, col), instance.model.at(1, col), instance.model.at(2, col),
                                     instance.model.at(3, col)};
        }
        const std::array<f32, 9> normal = normal_matrix(instance.model);
        record.normal[0] = Vec4{normal[0], normal[1], normal[2], 0.0f};
        record.normal[1] = Vec4{normal[3], normal[4], normal[5], 0.0f};
        record.normal[2] = Vec4{normal[6], normal[7], normal[8], 0.0f};
        records.push_back(record);
    }
}

Result<usize> Renderer3D::draw_skinned(rhi::Device& device, const Mat4& view_projection, const Vec3& camera_position,
                                       std::vector<render::SkinnedInstance>& instances, const FrameLighting& lighting) {
    FE_CHECK(skinned_shader_ != nullptr);

    last_skinned_draw_calls_ = 0;
    if (instances.empty()) {
        return usize{0};
    }

    // The per-instance transform stream. The shadow pass already built it when
    // this frame cast a skinned shadow; rebuilding is idempotent and cheap, and it
    // is the only path that guarantees the stream matches `instances` when the
    // shadow pass did not run.
    build_skinned_instance_stream();

    ensure_skinned_instance_capacity(device, static_cast<u32>(frame_skinned_instance_data_.size()));
    if (skinned_instance_buffer_ == nullptr) {
        return Error("renderer3d: skinned instance buffer unavailable");
    }
    skinned_instance_buffer_->update(
        std::span<const u8>(reinterpret_cast<const u8*>(frame_skinned_instance_data_.data()),
                            frame_skinned_instance_data_.size() * sizeof(Instance)));

    device.bind_shader(*skinned_shader_);
    skinned_shader_->set_mat4("u_view_projection", view_projection);
    set_lighting_uniforms(*skinned_shader_, camera_position, lighting);
    device.bind_texture(
        1, (lighting.shadows_enabled && shadow_target_ != nullptr && shadow_target_->depth_texture() != nullptr)
               ? *shadow_target_->depth_texture()
               : *default_texture_);

    // One draw per instance, because the palette is a uniform array and a uniform
    // cannot vary within an instanced draw. A rig's palette is uploaded before its
    // draw; a character and a prop of the same size therefore differ by one draw
    // call, which is the cost the uniform-palette design (ADR-0104) accepts.
    for (const render::SkinnedInstance& instance : instances) {
        auto gpu = acquire_skinned_mesh(device, *instance.mesh);
        if (!gpu) {
            FE_LOG_ERROR("renderer3d: %s", gpu.error().message().c_str());
            continue;
        }

        // The material, resolved through the same precedence rule the static path
        // uses (the component's surface, overridden by the mesh's imported
        // material). A skinned mesh with no material falls back to the component's
        // or the default white texel, exactly as a static one does.
        const MeshRenderer3D::Surface surface =
            instance.renderer != nullptr ? instance.renderer->resolve_surface() : MeshRenderer3D::Surface{};
        skinned_shader_->set_vec4("u_base_color", surface.material.base_color);
        skinned_shader_->set_float("u_specular", surface.specular);
        skinned_shader_->set_float("u_shininess", surface.shininess);
        skinned_shader_->set_vec3("u_emissive", surface.material.emissive);
        const Texture2D* base_texture =
            surface.material.base_color_texture.empty() ? nullptr : load_texture(surface.material.base_color_texture);
        const Texture2D* emissive_texture =
            surface.material.emissive_texture.empty() ? nullptr : load_texture(surface.material.emissive_texture);
        device.bind_texture(0, base_texture != nullptr ? base_texture->texture() : *default_texture_);
        skinned_shader_->set_float("u_emissive_map_enabled", emissive_texture != nullptr ? 1.0f : 0.0f);
        device.bind_texture(2, emissive_texture != nullptr ? emissive_texture->texture() : *default_texture_);

        // The palette. A missing palette is the bind pose: the identity, which
        // leaves the vertex where the bind shape put it. The array is uploaded at
        // the joint count, never more than the shader's cap; a larger rig is drawn
        // with its first kMaxPaletteJoints joints rather than reading past the end,
        // and reported once so the truncation is not silent.
        if (instance.palette != nullptr && instance.palette->size() > kMaxPaletteJoints) {
            report_oversized_palette(*instance.mesh, instance.palette->size());
        }
        const u32 joint_count = instance.palette != nullptr
                                    ? std::min<u32>(static_cast<u32>(instance.palette->size()), kMaxPaletteJoints)
                                    : 0;
        if (joint_count > 0) {
            skinned_shader_->set_mat4_array("u_palette", instance.palette->data(), joint_count);
        } else {
            // A single identity so the uniform array is complete even with no
            // joints; the shader reads index 0 only if a weight is non-zero, which
            // a bind-pose vertex's zero weights are not.
            const Mat4 identity = Mat4::identity();
            skinned_shader_->set_mat4_array("u_palette", &identity, 1);
        }

        device.bind_vertex_buffer(*gpu.value()->vertex_buffer);
        device.bind_instance_buffer(*skinned_instance_buffer_, static_cast<u32>(&instance - instances.data()) *
                                                                   static_cast<u32>(sizeof(Instance)));
        device.bind_index_buffer(*gpu.value()->index_buffer);
        device.draw_indexed_instanced(gpu.value()->index_count, 1);
        ++last_skinned_draw_calls_;
    }

    return static_cast<usize>(last_skinned_draw_calls_);
}

const Texture2D* Renderer3D::load_texture(const std::string& reference) {
    if (reference.empty()) {
        return nullptr;
    }
    if (const auto it = texture_cache_.find(reference); it != texture_cache_.end()) {
        return it->second.get();
    }
    // The memo, not the ResourceManager, owns a material texture (the manager
    // caches weakly and the reference is only a string), so the entry keeps the
    // shared_ptr alive for as long as the cache entry does. An entry that
    // resolved to null records the miss, so it is not retried for every instance
    // in the frame.
    std::shared_ptr<const Texture2D> texture;
    if (resources_ != nullptr) {
        if (auto loaded = resources_->load(reference)) {
            texture = std::dynamic_pointer_cast<const Texture2D>(loaded.value());
            if (texture == nullptr) {
                FE_LOG_WARN("renderer3d: material texture '%s' is not a Texture2D", reference.c_str());
            }
        } else {
            FE_LOG_WARN("renderer3d: could not load material texture '%s'", reference.c_str());
        }
    }

    const Texture2D* resolved = texture.get();
    texture_cache_.emplace(reference, std::move(texture));
    return resolved;
}

Error Renderer3D::ensure_shadow_target(rhi::Device& device) {
    if (shadow_target_ != nullptr || shadow_target_attempted_) {
        return Error::none();
    }
    shadow_target_attempted_ = true;

    rhi::RenderTargetDesc desc;
    desc.name = "renderer3d_shadow_map";
    // One tile per cascade, side by side: the pass switches viewport rather than
    // render target, and the shader offsets into the tile it selected.
    desc.width = kShadowMapSize * kShadowCascades;
    desc.height = kShadowMapSize;
    // Depth-only: the shadow pass writes no colour, so the target has no colour
    // attachment. A WebGL2 context otherwise rejects every draw into it, because
    // the shadow fragment shader has no output while the framebuffer has a colour
    // attachment enabled. See docs/adr/0076-3d-in-the-browser.md.
    desc.color = false;
    desc.depth = true;
    desc.sample_depth = true; // The main pass samples this depth; a renderbuffer would not do.
    auto target = device.create_render_target(desc);
    if (!target) {
        return target.error();
    }
    shadow_target_ = target.value();
    return Error::none();
}

void Renderer3D::extract(const Scene& scene, std::vector<render::MeshInstance>& out,
                         std::vector<const Mesh*>* meshes_seen, JobSystem* jobs) const {
    render::extract_meshes(scene, out, meshes_seen, jobs);
}

usize Renderer3D::cull(std::vector<render::MeshInstance>& out, const Frustum* frustum) {
    if (frustum == nullptr) {
        return 0;
    }
    if (out.empty()) {
        cull_bvh_valid_ = false;
        return 0;
    }

    refresh_cull_bvh(out);

    // Query the tree, then drop everything it did not return. The BVH's frustum
    // query is the same conservative `Frustum::intersects_aabb` test the flat
    // pass used, so the surviving set is identical; the tree only skips the
    // boxes a rejected subtree would have tested one by one. The output is
    // sorted by primitive index, which is the extraction order, so the draw
    // order - and therefore the rendered image - is unchanged.
    cull_bvh_.query(*frustum, cull_hits_);
    cull_visible_.assign(out.size(), false);
    for (u32 index : cull_hits_) {
        cull_visible_[index] = true;
    }

    const usize before = out.size();
    usize write = 0;
    for (usize read = 0; read < before; ++read) {
        if (cull_visible_[read]) {
            out[write++] = out[read];
        }
    }
    out.resize(write);
    return before - write;
}

usize Renderer3D::cull_flat(std::vector<render::MeshInstance>& out, const Frustum* frustum, JobSystem* jobs) {
    return render::cull_meshes(out, frustum, jobs);
}

bool Renderer3D::refresh_cull_bvh(const std::vector<render::MeshInstance>& instances) {
    const usize count = instances.size();
    if (cull_bvh_valid_ && cull_keys_.size() == count) {
        bool same = true;
        for (usize i = 0; i < count; ++i) {
            if (!(cull_keys_[i] == render::instance_key(instances[i]))) {
                same = false;
                break;
            }
        }
        if (same) {
            return false;
        }
    }

    cull_bounds_.resize(count);
    cull_keys_.resize(count);
    for (usize i = 0; i < count; ++i) {
        const render::MeshInstance& instance = instances[i];
        cull_bounds_[i] = Aabb{instance.world_min, instance.world_max};
        cull_keys_[i] = render::instance_key(instance);
    }
    cull_bvh_.build(cull_bounds_);
    cull_bvh_valid_ = true;
    ++bvh_rebuilds_;
    return true;
}

Result<usize> Renderer3D::render(rhi::Device& device, const Scene& scene, u32 viewport_width, u32 viewport_height,
                                 const PostProcessSettings& settings, JobSystem* jobs) {
    return render_frame(device, scene, viewport_width, viewport_height, settings, jobs);
}

Result<usize> Renderer3D::render_frame(rhi::Device& device, const Scene& scene, u32 viewport_width, u32 viewport_height,
                                       const PostProcessSettings& settings, JobSystem* jobs) {
    FE_CHECK(shader_ != nullptr);
    FE_CHECK(viewport_width > 0 && viewport_height > 0);

    last_draw_calls_ = 0;
    last_culled_ = 0;
    last_point_lights_ = 0;
    last_spot_lights_ = 0;
    last_shadow_casters_ = 0;
    last_shadow_cascades_ = 0;
    last_cascade_split_ = 0.0f;
    last_skinned_draw_calls_ = 0;
    last_skinned_culled_ = 0;
    last_transparent_draw_calls_ = 0;
    last_particles_ = 0;

    const Camera3D* camera = find_active_camera(scene);
    if (camera == nullptr) {
        return usize{0};
    }

    // --- 1. Extract --------------------------------------------------------
    // The working set is a member, so a steady-state frame refills the same
    // vectors instead of allocating. extract() clears `out` itself, and the
    // mesh-liveness list is cleared here, so a reused buffer cannot carry the
    // previous frame's entries into this one.
    std::vector<render::MeshInstance>& instances = frame_instances_;
    std::vector<const Mesh*>& meshes_seen = frame_meshes_seen_;
    meshes_seen.clear();
    extract(scene, instances, &meshes_seen, jobs);

    // --- 2. Cull -----------------------------------------------------------
    // Reject instances whose world AABB lies wholly outside the view. The test is
    // conservative, so this can only save a draw. The frustum is built from the
    // same matrix the draw uses, so the two can never disagree.
    const Mat4 view_projection = camera->view_projection(viewport_width, viewport_height);
    const Frustum frustum = Frustum::from_view_projection(view_projection);
    last_culled_ = static_cast<u32>(cull(instances, &frustum));

    // --- 2b. Extract and cull skinned instances ----------------------------
    // A separate pass, because a skinned instance has a different vertex stream
    // and a palette and cannot be batched with a static one. Extract then cull,
    // exactly as for static geometry, so a character off screen costs no draw.
    std::vector<render::SkinnedInstance>& skinned = frame_skinned_instances_;
    std::vector<const anim::SkinnedMesh*>& skinned_seen = frame_skinned_meshes_seen_;
    skinned_seen.clear();
    render::extract_skinned_meshes(scene, skinned, jobs);
    for (const render::SkinnedInstance& instance : skinned) {
        skinned_seen.push_back(instance.mesh);
    }
    last_skinned_culled_ = static_cast<u32>(render::cull_skinned_meshes(skinned, &frustum, jobs));

    // --- 2c. Extract and cull particle billboards --------------------------
    // The simulation belongs to ParticleSystem; this step only reads its output.
    // Culling is by each billboard's centre, so a particle straddling the frame
    // edge can vanish a frame early -- cheaper than carrying a radius through the
    // cull, and invisible at any particle size worth drawing.
    render::extract_particles(scene, frame_particles_);
    static_cast<void>(render::cull_particles(frame_particles_, &frustum));

    // --- 3. Draw -----------------------------------------------------------
    // The camera's background gradient and the post-processing chain are handled
    // inside `draw_chain`, because both need the destination target bound.
    return draw_chain(device, scene, view_projection, camera->eye(), *camera, instances, meshes_seen, settings,
                      viewport_width, viewport_height);
}

void Renderer3D::ensure_instance_capacity(rhi::Device& device, u32 count) {
    // The record uploaded per instance must match the vertex layout's stride
    // exactly, or the GPU reads each instance's transform shifted by the
    // difference and every instance after the first is wrong.
    static_assert(sizeof(Instance) == 7 * 16, "Instance must match VertexLayout::mesh_instance()'s stride");

    if (count <= instance_capacity_) {
        return;
    }
    const u32 new_capacity = std::max(count, instance_capacity_ * 2);
    auto buffer = device.create_vertex_buffer(rhi::VertexLayout::mesh_instance(),
                                              new_capacity * static_cast<u32>(sizeof(Instance)));
    if (!buffer) {
        FE_LOG_ERROR("renderer3d: could not grow instance buffer: %s", buffer.error().message().c_str());
        return;
    }
    instance_buffer_ = buffer.value();
    instance_capacity_ = new_capacity;
}

void Renderer3D::build_batches(const std::vector<render::MeshInstance>& instances) {
    frame_instance_data_.clear();
    frame_batches_.clear();
    if (instances.empty()) {
        return;
    }
    frame_instance_data_.reserve(instances.size());

    for (const render::MeshInstance& instance : instances) {
        if (instance.mesh == nullptr || instance.renderer == nullptr) {
            continue;
        }

        // Resolve the surface and its textures here, once per instance, so the
        // batch key below compares resolved values rather than re-resolving.
        // `load_texture` memoises per frame, so two instances of one material
        // share the lookup and compare equal.
        const MeshRenderer3D::Surface surface = instance.renderer->resolve_surface();
        const Texture2D* base_texture = nullptr;
        if (!surface.material.base_color_texture.empty()) {
            base_texture = load_texture(surface.material.base_color_texture);
        }
        if (base_texture == nullptr && instance.renderer->has_texture()) {
            base_texture = dynamic_cast<const Texture2D*>(instance.renderer->texture.get());
        }
        const Texture2D* emissive_texture = nullptr;
        if (!surface.material.emissive_texture.empty()) {
            emissive_texture = load_texture(surface.material.emissive_texture);
        }

        // The per-instance transform record. The model matrix's columns go in
        // directly; the normal matrix is derived on the CPU (one inverse per
        // instance) rather than per vertex in the shader.
        Instance record{};
        for (usize col = 0; col < 4; ++col) {
            record.model[col] = Vec4{instance.model.at(0, col), instance.model.at(1, col), instance.model.at(2, col),
                                     instance.model.at(3, col)};
        }
        const std::array<f32, 9> normal = normal_matrix(instance.model);
        record.normal[0] = Vec4{normal[0], normal[1], normal[2], 0.0f};
        record.normal[1] = Vec4{normal[3], normal[4], normal[5], 0.0f};
        record.normal[2] = Vec4{normal[6], normal[7], normal[8], 0.0f};
        frame_instance_data_.push_back(record);

        const BatchState state{surface.material.base_color, surface.specular, surface.shininess,
                               surface.material.emissive};
        const bool casts_shadow = instance.renderer->casts_shadow;
        const bool transparent = instance.transparent;

        // Extend the open batch when this instance matches it in every per-batch
        // field and is contiguous in the instance stream, so the run is one draw.
        if (!frame_batches_.empty()) {
            Batch& open = frame_batches_.back();
            if (open.mesh == instance.mesh && open.base_texture == base_texture &&
                open.emissive_texture == emissive_texture && open.state == state && open.casts_shadow == casts_shadow &&
                open.transparent == transparent) {
                open.instance_count += 1;
                continue;
            }
        }
        frame_batches_.push_back(Batch{instance.mesh, base_texture, emissive_texture, state, casts_shadow, transparent,
                                       static_cast<u32>(frame_instance_data_.size() - 1), 1});
    }
}

void Renderer3D::sort_for_transparency(std::vector<render::MeshInstance>& instances, const Vec3& camera_position) {
    // Opaque first, so their batches form one contiguous run, then the translucent
    // tail. A stable partition leaves the extraction order inside each group
    // untouched, so a scene with nothing translucent keeps exactly the array it
    // had before this pass existed -- which is what makes the change additive for
    // every existing scene and every existing test.
    const auto first_translucent = std::stable_partition(
        instances.begin(), instances.end(), [](const render::MeshInstance& instance) { return !instance.transparent; });
    if (first_translucent == instances.end()) {
        return;
    }

    // Back to front: a translucent surface blends over what is already in the
    // frame, so the one behind it must be drawn first. The key is the squared
    // distance from the camera to the instance's world-bounds centre, a cheap
    // proxy for depth that needs no per-instance projection. A tie keeps
    // extraction order, which is why the sort is stable.
    const auto distance_squared = [&camera_position](const render::MeshInstance& instance) {
        const Vec3 centre = (instance.world_min + instance.world_max) * 0.5f;
        return (centre - camera_position).length_squared();
    };
    std::stable_sort(first_translucent, instances.end(),
                     [&distance_squared](const render::MeshInstance& a, const render::MeshInstance& b) {
                         return distance_squared(a) > distance_squared(b);
                     });
}

void Renderer3D::set_lighting_uniforms(rhi::Shader& shader, const Vec3& camera_position,
                                       const FrameLighting& lighting) {
    // Sampler slots. `u_texture` is the base-colour map (slot 0), `u_emissive_map`
    // the emissive map (slot 2); the shadow map takes slot 1. The slots are set
    // once per pass, so the per-batch code below only binds textures to them.
    shader.set_int("u_texture", 0);
    shader.set_int("u_emissive_map", 2);
    shader.set_vec3("u_light_color", lighting.color);
    shader.set_float("u_light_intensity", lighting.intensity);
    shader.set_float("u_ambient", lighting.ambient);
    shader.set_vec3("u_light_direction", lighting.direction);
    shader.set_vec3("u_camera_position", camera_position);

    // Shadow uniforms. When no shadow map exists (no directional light, or
    // creation failed) the light-space matrix stays identity and the map is never
    // sampled usefully, but the sampler still needs a bound texture to be a
    // complete program, so bind the white texel.
    shader.set_int("u_shadow_map", 1);
    shader.set_mat4_array("u_light_space_matrix", lighting.light_space_matrix.data(), kShadowCascades);
    shader.set_float("u_shadow_bias", kShadowBias);
    shader.set_vec3("u_camera_forward", lighting.camera_forward);
    shader.set_float("u_cascade_split", lighting.cascade_split);
    // The device is reached through the shader's own owner, so the texture
    // binding is done by the caller; this sets the uniform state only. See the
    // bind sites in draw() and draw_skinned().

    // Pack and upload the bounded light arrays. Only the first point_count /
    // spot_count elements are read by the shader, whose loop is bounded by the
    // count uniforms.
    const u32 point_count = last_point_lights_;
    shader.set_int("u_point_light_count", static_cast<i32>(point_count));
    shader.set_vec4_array("u_point_light_position_range", point_light_data_.data(), point_count);
    shader.set_vec4_array("u_point_light_color_intensity", point_color_data_.data(), point_count);

    const u32 spot_count = last_spot_lights_;
    shader.set_int("u_spot_light_count", static_cast<i32>(spot_count));
    // The direction half of the block must be uploaded even when only the first
    // few entries are used, so always send the full 2*max block; the shader reads
    // it only for i < spot_count.
    shader.set_vec4_array("u_spot_light_data", spot_light_data_.data(), spot_count == 0 ? 0 : kMaxSpotLights * 2);
    shader.set_vec4_array("u_spot_light_color_intensity", spot_color_data_.data(), spot_count);
    shader.set_vec4_array("u_spot_light_cone", spot_cone_data_.data(), spot_count);
}

Result<usize> Renderer3D::draw(rhi::Device& device, const Scene& scene, const Mat4& view_projection,
                               const Vec3& camera_position, std::vector<render::MeshInstance>& instances,
                               const std::vector<const Mesh*>& meshes_seen) {
    FrameLighting lighting;
    prepare_frame(device, scene, instances, camera_position, lighting);
    if (instances.empty() && frame_skinned_instances_.empty()) {
        return usize{0};
    }
    // The shadow and main passes are this renderer's two steps, run in that
    // order. `render_frame` declares them on a graph; this direct path keeps
    // them as calls, so an existing caller's behaviour is unchanged.
    draw_shadow_pass(device, view_projection, camera_position, instances, lighting);
    Result<usize> main = draw_main_pass(device, view_projection, camera_position, nullptr, instances, lighting);
    if (!main) {
        return main.error();
    }
    sweep_dead_buffers(meshes_seen);
    return main.value();
}

usize Renderer3D::prepare_frame(rhi::Device& device, const Scene& scene, std::vector<render::MeshInstance>& instances,
                                const Vec3& camera_position, FrameLighting& lighting) {
    FE_CHECK(shader_ != nullptr);

    last_draw_calls_ = 0;
    last_point_lights_ = 0;
    last_spot_lights_ = 0;
    last_shadow_casters_ = 0;
    last_shadow_cascades_ = 0;
    last_cascade_split_ = 0.0f;
    // The material-texture memo is per frame: a reference re-resolves each frame
    // so a hot-reloaded map is picked up, while instances within one frame share
    // the single lookup.
    texture_cache_.clear();

    if (instances.empty() && frame_skinned_instances_.empty()) {
        // Nothing to draw this frame. The batched working set is cleared here
        // rather than left as the previous frame left it, because the main pass
        // reads `frame_batches_` and would otherwise redraw last frame's batches.
        frame_instance_data_.clear();
        frame_batches_.clear();
        return 0;
    }

    // --- Frame constants ----------------------------------------------------
    // Point and spot lights need their node's world position (and, for spots,
    // the node's forward direction), so they are collected here where the node
    // is still in hand. Brightness is intensity scaled by range, which is a
    // cheap proxy for influence used only to pick which lights survive the cap.
    // The lists are members reused across frames, so the gather does not
    // allocate once their capacity covers the scene's lights.
    std::vector<PointLightEntry>& point_lights = frame_point_lights_;
    std::vector<SpotLightEntry>& spot_lights = frame_spot_lights_;
    point_lights.clear();
    spot_lights.clear();

    // The directional light is picked up on the same walk as the point and spot
    // lights rather than by a second traversal: the loop already visits every
    // node, and a directional light is global, so the first one seen is the one
    // used. One walk instead of two, and the scene's node count is what a frame
    // pays for it.
    const DirectionalLight3D* light = nullptr;

    scene.for_each_node([&](Node& node) {
        const Mat4 world = node.world_matrix();
        if (light == nullptr) {
            light = node.get_component<DirectionalLight3D>();
        }
        if (const auto* point = node.get_component<PointLight3D>(); point != nullptr) {
            point_lights.push_back(PointLightEntry{transform_point(world, Vec3{}), point->color, point->intensity,
                                                   point->range, point->intensity * point->range});
        }
        if (const auto* spot = node.get_component<SpotLight3D>(); spot != nullptr) {
            // The node's local -Z is the cone axis, matching Camera3D. A zero
            // scale would collapse it, so normalise defensively.
            const Vec3 forward = transform_direction(world, Vec3{0.0f, 0.0f, -1.0f}).normalized();
            const f32 inner = radians(spot->inner_cone_degrees);
            const f32 outer = radians(spot->outer_cone_degrees);
            spot_lights.push_back(SpotLightEntry{transform_point(world, Vec3{}), forward, spot->color, spot->intensity,
                                                 spot->range, std::cos(inner), std::cos(outer),
                                                 spot->intensity * spot->range});
        }
    });

    const auto by_brightness = [](const auto& a, const auto& b) { return a.brightness > b.brightness; };
    if (point_lights.size() > kMaxPointLights) {
        std::nth_element(point_lights.begin(), point_lights.begin() + kMaxPointLights, point_lights.end(),
                         by_brightness);
        point_lights.resize(kMaxPointLights);
    }
    if (spot_lights.size() > kMaxSpotLights) {
        std::nth_element(spot_lights.begin(), spot_lights.begin() + kMaxSpotLights, spot_lights.end(), by_brightness);
        spot_lights.resize(kMaxSpotLights);
    }

    last_point_lights_ = static_cast<u32>(point_lights.size());
    last_spot_lights_ = static_cast<u32>(spot_lights.size());

    // Pack the bounded light arrays once per frame, before either lit pass, so
    // the static and skinned passes upload identical lighting. Only the first
    // point_count / spot_count elements are read by the shader.
    for (u32 i = 0; i < last_point_lights_; ++i) {
        const PointLightEntry& entry = point_lights[i];
        point_light_data_[i] = Vec4{entry.position, entry.range};
        point_color_data_[i] = Vec4{entry.color, entry.intensity};
    }
    for (u32 i = 0; i < last_spot_lights_; ++i) {
        const SpotLightEntry& entry = spot_lights[i];
        // The position/direction arrays are uploaded as one 16-element block:
        // [0, 8) positions + ranges, [8, 16) directions.
        spot_light_data_[i] = Vec4{entry.position, entry.range};
        spot_light_data_[i + kMaxSpotLights] = Vec4{entry.direction, 0.0f};
        spot_color_data_[i] = Vec4{entry.color, entry.intensity};
        spot_cone_data_[i] = Vec4{entry.cos_inner, entry.cos_outer, 0.0f, 0.0f};
    }

    const Vec3 light_direction = light != nullptr ? light->direction.normalized() : Vec3{0.0f, -1.0f, 0.0f};
    const Vec3 light_color = light != nullptr ? light->color : Vec3{1.0f, 1.0f, 1.0f};
    const f32 light_intensity = light != nullptr ? light->intensity : 1.0f;
    const f32 ambient = light != nullptr ? light->ambient : 0.15f;

    // --- 3a. Batch ----------------------------------------------------------
    // Group the visible instances into one instanced draw per (mesh, textures,
    // batch state) run and build the per-instance transform stream they read.
    // Done once, before either pass, so the shadow pass and the main pass draw
    // the same batches from the same upload. The count is every instance that
    // resolved to a drawable record, so it matches the pre-instancing per-instance
    // loop; a batch whose mesh fails to acquire is subtracted below.
    sort_for_transparency(instances, camera_position);

    build_batches(instances);
    u32 drawn = static_cast<u32>(frame_instance_data_.size());
    if (drawn > 0) {
        ensure_instance_capacity(device, drawn);
    }
    const bool instancing_ready = drawn > 0 && instance_buffer_ != nullptr;
    if (instancing_ready) {
        instance_buffer_->update(std::span<const u8>(reinterpret_cast<const u8*>(frame_instance_data_.data()),
                                                     frame_instance_data_.size() * sizeof(Instance)));
    }

    lighting.direction = light_direction;
    lighting.color = light_color;
    lighting.intensity = light_intensity;
    lighting.ambient = ambient;
    lighting.light_space_matrix = {Mat4::identity(), Mat4::identity()};
    lighting.cascade_split = 0.0f;
    // The shadow pass runs only when there is a directional light and something
    // to cast from. `instancing_ready` and the skinned set are the frame's
    // drawable geometry, so a lit but empty frame skips the map entirely.
    lighting.shadows_enabled = light != nullptr && (instancing_ready || !frame_skinned_instances_.empty());
    return static_cast<usize>(drawn);
}

void Renderer3D::draw_shadow_pass(rhi::Device& device, const Mat4& view_projection, const Vec3& camera_position,
                                  std::vector<render::MeshInstance>& instances, FrameLighting& lighting) {
    // Render the visible geometry's depth from the light's point of view, once
    // per cascade, then restore the target the pass found bound. The cascades
    // partition the depth band the casters occupy, so the atlas spends its
    // resolution on the geometry nearest the camera instead of spreading it
    // evenly over the whole scene (ADR-0161).
    //
    // The light is resolved in `prepare_frame`; a scene with no directional light
    // resolves `shadows_enabled` false and this pass is a no-op.
    if (!lighting.shadows_enabled) {
        return;
    }

    Vec3 caster_min{};
    Vec3 caster_max{};
    bool bounds_valid = false;
    const auto expand = [&](const Vec3& world_min, const Vec3& world_max) {
        if (!bounds_valid) {
            caster_min = world_min;
            caster_max = world_max;
            bounds_valid = true;
            return;
        }
        caster_min.x = std::min(caster_min.x, world_min.x);
        caster_min.y = std::min(caster_min.y, world_min.y);
        caster_min.z = std::min(caster_min.z, world_min.z);
        caster_max.x = std::max(caster_max.x, world_max.x);
        caster_max.y = std::max(caster_max.y, world_max.y);
        caster_max.z = std::max(caster_max.z, world_max.z);
    };
    // Only casters shape the fit. A large non-casting ground plane would
    // otherwise stretch the map over the whole world and starve the objects
    // whose shadows are actually seen, which is the reason the flag exists.
    for (const render::MeshInstance& instance : instances) {
        if (instance.renderer == nullptr || !instance.renderer->casts_shadow || instance.transparent) {
            continue;
        }
        // The world AABB was computed once at extraction; the fit reads it rather
        // than re-expanding the mesh's local bounds.
        expand(instance.world_min, instance.world_max);
    }
    // A skinned caster contributes its *posed* bounds, so a character whose arm
    // swings outside the static geometry still fits the shadow box.
    for (const render::SkinnedInstance& instance : frame_skinned_instances_) {
        if (instance.renderer != nullptr && !instance.renderer->casts_shadow) {
            continue;
        }
        expand(instance.world_min, instance.world_max);
    }
    if (!bounds_valid) {
        return;
    }

    if (Error error = ensure_shadow_target(device); error) {
        FE_LOG_ERROR("renderer3d: shadow map unavailable: %s", error.message().c_str());
        return;
    }

    const Mat4 inverse_view_projection = inverse(view_projection);
    const Vec3 forward = camera_forward(inverse_view_projection);
    // The band runs from the camera's near plane to the farthest caster: nothing
    // nearer is drawn, and nothing farther can cast into the view. Starting at
    // the nearest caster instead would be tighter, but a caster whose AABB
    // straddles the camera plane reports a nearest depth of zero and would hand
    // the near cascade almost no world to cover.
    const f32 camera_near = shadow_camera_near_depth(inverse_view_projection, camera_position);
    const auto caster_depths = shadow_caster_depth_range(caster_min, caster_max, camera_position, forward);
    const f32 band_near = std::max(std::max(camera_near, kShadowNearDepth), caster_depths.first);
    const f32 band_far = std::max(caster_depths.second, band_near);
    const auto splits = shadow_cascade_splits(band_near, band_far, kShadowSplitLambda);

    // The skinned instance stream is built once for the frame rather than per
    // cascade: the shadow pass and the lit pass must upload identical transforms,
    // and a second cascade would only rebuild the same records.
    if (!frame_skinned_instances_.empty()) {
        build_skinned_instance_stream();
        ensure_skinned_instance_capacity(device, static_cast<u32>(frame_skinned_instance_data_.size()));
        if (skinned_instance_buffer_ != nullptr) {
            skinned_instance_buffer_->update(
                std::span<const u8>(reinterpret_cast<const u8*>(frame_skinned_instance_data_.data()),
                                    frame_skinned_instance_data_.size() * sizeof(Instance)));
        }
    }

    rhi::RenderTarget* previous_target = device.bound_render_target();
    device.bind_render_target(shadow_target_.get());
    // One clear for the whole atlas. A per-cascade clear would wipe the cascade
    // just drawn, because a clear is bounded by the scissor and not the viewport.
    shadow_target_->clear(rhi::ClearColor{1.0f, 1.0f, 1.0f, 1.0f}); // Far depth.
    device.set_depth_test(true);

    for (u32 cascade = 0; cascade < kShadowCascades; ++cascade) {
        // Each cascade owns one tile of the atlas, so the pass switches viewport
        // rather than render target: the shader samples one texture and offsets
        // into the tile it selected.
        device.set_viewport(cascade * kShadowMapSize, 0, kShadowMapSize, kShadowMapSize);
        const Mat4 light_view_projection =
            shadow_cascade_light_matrix(inverse_view_projection, camera_position, splits[cascade], splits[cascade + 1],
                                        caster_min, caster_max, lighting.direction);
        lighting.light_space_matrix[cascade] = light_view_projection;

        device.bind_shader(*shadow_shader_);
        shadow_shader_->set_mat4("u_light_view_projection", light_view_projection);

        for (const Batch& batch : frame_batches_) {
            if (!batch.casts_shadow || batch.transparent) {
                continue; // Draws in the main pass, casts nothing.
            }
            auto gpu = acquire_mesh(device, *batch.mesh);
            if (!gpu) {
                continue;
            }
            device.bind_vertex_buffer(*gpu.value()->vertex_buffer);
            device.bind_instance_buffer(*instance_buffer_, batch.instance_offset * static_cast<u32>(sizeof(Instance)));
            device.bind_index_buffer(*gpu.value()->index_buffer);
            device.draw_indexed_instanced(gpu.value()->index_count, batch.instance_count);
            last_shadow_casters_ += batch.instance_count;
        }

        // Skinned casters, with the skinned depth shader so a character casts the
        // shadow of its pose. The instance stream was built above; only the
        // per-instance palette changes from draw to draw.
        if (skinned_instance_buffer_ != nullptr) {
            device.bind_shader(*skinned_shadow_shader_);
            skinned_shadow_shader_->set_mat4("u_light_view_projection", light_view_projection);
            for (usize i = 0; i < frame_skinned_instances_.size(); ++i) {
                const render::SkinnedInstance& instance = frame_skinned_instances_[i];
                if (instance.renderer != nullptr && !instance.renderer->casts_shadow) {
                    continue; // Draws in the main pass, casts nothing.
                }
                auto gpu = acquire_skinned_mesh(device, *instance.mesh);
                if (!gpu) {
                    continue;
                }
                const u32 joint_count =
                    instance.palette != nullptr
                        ? std::min<u32>(static_cast<u32>(instance.palette->size()), kMaxPaletteJoints)
                        : 0;
                if (instance.palette != nullptr && instance.palette->size() > kMaxPaletteJoints) {
                    report_oversized_palette(*instance.mesh, instance.palette->size());
                }
                if (joint_count > 0) {
                    skinned_shadow_shader_->set_mat4_array("u_palette", instance.palette->data(), joint_count);
                } else {
                    const Mat4 identity = Mat4::identity();
                    skinned_shadow_shader_->set_mat4_array("u_palette", &identity, 1);
                }
                device.bind_vertex_buffer(*gpu.value()->vertex_buffer);
                device.bind_instance_buffer(*skinned_instance_buffer_,
                                            static_cast<u32>(i) * static_cast<u32>(sizeof(Instance)));
                device.bind_index_buffer(*gpu.value()->index_buffer);
                device.draw_indexed_instanced(gpu.value()->index_count, 1);
                last_shadow_casters_ += 1;
            }
        }
    }

    // Restore whatever the caller had bound (the window, or an editor panel's
    // target) so the main pass draws where it was meant to.
    device.bind_render_target(previous_target);
    lighting.shadows_enabled = true;
    lighting.cascade_split = splits[1];
    lighting.camera_forward = forward;
    last_shadow_cascades_ = kShadowCascades;
    last_cascade_split_ = splits[1];
}

Result<usize> Renderer3D::draw_main_pass(rhi::Device& device, const Mat4& view_projection, const Vec3& camera_position,
                                         const Camera3D* camera, std::vector<render::MeshInstance>& instances,
                                         const FrameLighting& lighting) {
    // The camera's background gradient is geometry-independent, so it draws even
    // when the cull left nothing: an empty scene under a sky is a valid frame.
    // It is drawn first, into whichever image the pass draws into, so it lands
    // behind the geometry and is graded along with it. The direct `draw()` path
    // has no camera and so draws no background.
    if (camera != nullptr && camera->background_enabled) {
        if (Error error = ensure_background(device); error) {
            FE_LOG_ERROR("renderer3d: background unavailable: %s", error.message().c_str());
        } else {
            draw_background(device, *camera);
        }
    }

    // --- 4. Pass state ------------------------------------------------------
    // The opaque pass is depth-tested and writes depth. Blending is disabled so
    // draws do not depend on order; the transparent pass below turns it on and
    // depth writing off.
    device.set_blend_mode(rhi::BlendMode::None);
    device.set_depth_test(true);
    device.set_depth_write(true);
    last_transparent_draw_calls_ = 0;
    last_particles_ = 0;

    const bool shadows_enabled = lighting.shadows_enabled;
    const bool instancing_ready = !frame_batches_.empty() && instance_buffer_ != nullptr;
    u32 drawn = static_cast<u32>(frame_instance_data_.size());

    // Binds the lit static shader and every frame-level uniform it needs. Called
    // once before the opaque batches and again before the translucent ones,
    // because the skinned pass in between rebinds its own shader.
    const auto bind_static_pass = [&]() {
        device.bind_shader(*shader_);
        shader_->set_mat4("u_view_projection", view_projection);
        set_lighting_uniforms(*shader_, camera_position, lighting);
        // The shadow map's texture binding is device state, not a uniform, so it
        // is bound here beside the sampler uniform the helper set.
        device.bind_texture(1,
                            (shadows_enabled && shadow_target_ != nullptr && shadow_target_->depth_texture() != nullptr)
                                ? *shadow_target_->depth_texture()
                                : *default_texture_);
    };

    if (!instances.empty()) {
        bind_static_pass();
    }

    // --- 5. Draw ------------------------------------------------------------
    // One instanced draw per batch. Every per-batch uniform is set once here;
    // the per-instance transform and normal matrix arrive in the instance
    // stream, so nothing below varies per instance.
    if (instancing_ready) {
        for (const Batch& batch : frame_batches_) {
            if (batch.transparent) {
                continue; // Drawn after the opaque surfaces, in 5c.
            }
            auto gpu = acquire_mesh(device, *batch.mesh);
            if (!gpu) {
                FE_LOG_ERROR("renderer3d: %s", gpu.error().message().c_str());
                drawn -= batch.instance_count;
                continue;
            }

            shader_->set_vec4("u_base_color", batch.state.base_color);
            shader_->set_float("u_specular", batch.state.specular);
            shader_->set_float("u_shininess", batch.state.shininess);
            shader_->set_vec3("u_emissive", batch.state.emissive);

            device.bind_texture(0, batch.base_texture != nullptr ? batch.base_texture->texture() : *default_texture_);
            shader_->set_float("u_emissive_map_enabled", batch.emissive_texture != nullptr ? 1.0f : 0.0f);
            device.bind_texture(2, batch.emissive_texture != nullptr ? batch.emissive_texture->texture()
                                                                     : *default_texture_);

            device.bind_vertex_buffer(*gpu.value()->vertex_buffer);
            device.bind_instance_buffer(*instance_buffer_, batch.instance_offset * static_cast<u32>(sizeof(Instance)));
            device.bind_index_buffer(*gpu.value()->index_buffer);
            device.draw_indexed_instanced(gpu.value()->index_count, batch.instance_count);
            ++last_draw_calls_;
        }
    }

    // --- 5b. Skinned pass ---------------------------------------------------
    // The same frame, drawn for the skinned instances: a separate shader, a
    // separate vertex layout, and the palette uploaded per instance. Drawn after
    // the static pass so the two share the frame's lighting and shadow map, which
    // were set up above. Nothing here touches the static batching, so a scene
    // with no skinned character pays only the extract and the empty-array check.
    last_skinned_draw_calls_ = 0;
    if (!frame_skinned_instances_.empty()) {
        const Result<usize> skinned_drawn =
            draw_skinned(device, view_projection, camera_position, frame_skinned_instances_, lighting);
        if (!skinned_drawn) {
            FE_LOG_ERROR("renderer3d: skinned pass failed: %s", skinned_drawn.error().message().c_str());
        }
    }

    // --- 5c. Transparent pass ----------------------------------------------
    // Drawn after every opaque surface, static and skinned, with blending on and
    // depth writing off. The batches arrive farthest-first because
    // `sort_for_transparency` ordered the instances, so the run order is the
    // blend order: the surface behind is already in the frame before the one in
    // front composites over it. Depth *testing* stays on, so an opaque wall in
    // front still hides a translucent surface behind it; depth *writing* is off
    // so a farther translucent surface drawn later in the same run is not
    // rejected by a nearer one.
    bool transparent_state = false;
    if (instancing_ready) {
        for (const Batch& batch : frame_batches_) {
            if (!batch.transparent) {
                continue;
            }
            if (!transparent_state) {
                // The skinned pass above rebound its own shader and uniforms, so
                // the lit static pass is bound again here.
                bind_static_pass();
                device.set_blend_mode(rhi::BlendMode::Alpha);
                device.set_depth_write(false);
                transparent_state = true;
            }
            auto gpu = acquire_mesh(device, *batch.mesh);
            if (!gpu) {
                FE_LOG_ERROR("renderer3d: %s", gpu.error().message().c_str());
                drawn -= batch.instance_count;
                continue;
            }

            shader_->set_vec4("u_base_color", batch.state.base_color);
            shader_->set_float("u_specular", batch.state.specular);
            shader_->set_float("u_shininess", batch.state.shininess);
            shader_->set_vec3("u_emissive", batch.state.emissive);

            device.bind_texture(0, batch.base_texture != nullptr ? batch.base_texture->texture() : *default_texture_);
            shader_->set_float("u_emissive_map_enabled", batch.emissive_texture != nullptr ? 1.0f : 0.0f);
            device.bind_texture(2, batch.emissive_texture != nullptr ? batch.emissive_texture->texture()
                                                                     : *default_texture_);

            device.bind_vertex_buffer(*gpu.value()->vertex_buffer);
            device.bind_instance_buffer(*instance_buffer_, batch.instance_offset * static_cast<u32>(sizeof(Instance)));
            device.bind_index_buffer(*gpu.value()->index_buffer);
            device.draw_indexed_instanced(gpu.value()->index_count, batch.instance_count);
            ++last_draw_calls_;
            last_transparent_draw_calls_ += batch.instance_count;
        }
    }
    if (transparent_state) {
        device.set_blend_mode(rhi::BlendMode::None);
        device.set_depth_write(true);
    }

    // --- 5d. Particle pass --------------------------------------------------
    // The frame's particle billboards, on the same transparent state: blending
    // on, depth writing off, depth testing against the opaque geometry so a
    // particle behind a wall is hidden. They draw last because a particle is
    // translucent content that composites over everything opaque.
    if (camera != nullptr) {
        draw_particles(device, view_projection, *camera);
    }

    return static_cast<usize>(drawn);
}

void Renderer3D::sweep_dead_buffers(const std::vector<const Mesh*>& meshes_seen) {
    // A buffer is stale when its mesh is no longer referenced by any renderer in
    // the scene. Liveness is checked against every mesh seen this frame, culled
    // or not, so a mesh that is simply off screen keeps its buffers.
    //
    // The live set is built once into a hash set rather than scanned per cached
    // buffer: the previous `none_of` was O(cached meshes x instances seen), which
    // is quadratic in a scene whose meshes are all on screen -- exactly the
    // steady state of a game, and the case that must stay cheap. Building the set
    // is O(instances seen) and the sweep is O(cached meshes).
    live_meshes_.clear();
    live_meshes_.reserve(meshes_seen.size() * 2);
    live_meshes_.insert(meshes_seen.begin(), meshes_seen.end());
    std::erase_if(buffers_, [&](const auto& entry) { return !live_meshes_.contains(entry.first); });

    // The same sweep for the skinned buffers, against the skinned meshes seen.
    live_skinned_meshes_.clear();
    live_skinned_meshes_.reserve(frame_skinned_meshes_seen_.size() * 2);
    live_skinned_meshes_.insert(frame_skinned_meshes_seen_.begin(), frame_skinned_meshes_seen_.end());
    std::erase_if(skinned_buffers_, [&](const auto& entry) { return !live_skinned_meshes_.contains(entry.first); });
}

Result<usize> Renderer3D::draw_chain(rhi::Device& device, const Scene& scene, const Mat4& view_projection,
                                     const Vec3& camera_position, const Camera3D& camera,
                                     std::vector<render::MeshInstance>& instances,
                                     const std::vector<const Mesh*>& meshes_seen, const PostProcessSettings& settings,
                                     u32 width, u32 height) {
    // Resolve the target the main pass should draw into and bind it *before* the
    // background, so the gradient lands in the same image as the geometry: the
    // caller's target when the chain is off, the off-screen target when it is on.
    // A failed setup leaves the caller's target bound, so the frame is ungraded
    // rather than lost.
    rhi::RenderTarget* caller_target = device.bound_render_target();
    rhi::RenderTarget* pass_target = caller_target;
    bool chained = false;
    if (settings.enabled) {
        if (Error target_error = ensure_scene_target(device, width, height); target_error) {
            FE_LOG_ERROR("renderer3d: post-process target unavailable: %s", target_error.message().c_str());
        } else if (Error shader_error = post_process_.initialize(device); shader_error) {
            FE_LOG_ERROR("renderer3d: post-process shader unavailable: %s", shader_error.message().c_str());
        } else {
            pass_target = scene_target_.get();
            chained = true;
            device.bind_render_target(pass_target);
            scene_target_->clear(rhi::ClearColor{0.0f, 0.0f, 0.0f, 1.0f});
        }
    }

    // The frame's passes as a graph (ADR-0100): shadow, then main, then post.
    // The ordering the class used to express by calling the passes in sequence is
    // now data, so a caller can extend or reorder it and every pass is timed as
    // `pass/<name>`. The graph is rebuilt per call because it is cheap (three
    // names) and a long-lived one would have to be guarded against concurrent
    // frames for no gain.
    //
    // The graph is always the same three passes regardless of whether the chain
    // is on: `shadow` and `main` always run, and `post` runs when the off-screen
    // target is in play. Declaring all three keeps the run order -- and so the
    // profiler's pass breakdown -- identical frame to frame, rather than changing
    // shape with a setting. A pass the graph culled would not run, so the
    // resource declarations are what keep the passes live.
    render::RenderGraph graph;
    if (Result<void> added = graph.add_pass("shadow"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_pass("main"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_pass("post"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_dependency("main", "shadow"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_dependency("post", "main"); !added) {
        return added.error();
    }
    // Declare the flow so the graph can reason about it (ADR-0110): the shadow
    // pass writes the shadow map, which the main pass reads; the main pass writes
    // the frame image the post pass grades into the caller's target. Every pass
    // stays live -- the shadow map is read by the main pass, the frame image is
    // read by the post pass, and the post pass writes the marked output -- which
    // is exactly the order the dependencies already express. The declarations
    // cost nothing at run time and make the seam able to cull a pass whose output
    // nobody reads as the chain grows.
    if (Result<void> added = graph.add_write("shadow", "shadow_map"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_read("main", "shadow_map"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_write("main", "backbuffer"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_read("post", "backbuffer"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.add_write("post", "swapchain"); !added) {
        return added.error();
    }
    if (Result<void> added = graph.mark_output("swapchain"); !added) {
        return added.error();
    }

    FrameLighting lighting;
    prepare_frame(device, scene, instances, camera_position, lighting);
    Result<usize> drawn = usize{0};
    const Error error = graph.run([&](std::string_view pass) -> Error {
        if (pass == "shadow") {
            draw_shadow_pass(device, view_projection, camera_position, instances, lighting);
            return Error::none();
        }
        if (pass == "main") {
            Result<usize> main = draw_main_pass(device, view_projection, camera_position, &camera, instances, lighting);
            if (!main) {
                return main.error();
            }
            drawn = main.value();
            return Error::none();
        }
        // "post": the graph guarantees this is the only other pass. Grade the
        // off-screen scene image into the caller's target, restoring it first so
        // the post quad draws where the frame began.
        if (chained) {
            device.bind_render_target(caller_target);
            if (Error post_error = post_process_.apply(device, *scene_target_->color_texture(), settings); post_error) {
                FE_LOG_ERROR("renderer3d: post-process pass failed: %s", post_error.message().c_str());
            } else {
                ++post_process_frames_;
            }
        }
        return Error::none();
    });
    if (error) {
        return error;
    }

    // The mesh-buffer sweep is the frame's last step, after every pass that draws
    // from the cache.
    sweep_dead_buffers(meshes_seen);
    return drawn;
}

Error Renderer3D::ensure_scene_target(rhi::Device& device, u32 width, u32 height) {
    if (scene_target_ != nullptr && scene_target_width_ == width && scene_target_height_ == height) {
        return Error::none();
    }

    rhi::RenderTargetDesc desc;
    desc.name = "renderer3d_scene";
    desc.width = width;
    desc.height = height;
    desc.depth = true; // The 3D pass depth-tests, so the off-screen image needs a depth buffer.
    desc.sample_depth = false;
    auto target = device.create_render_target(desc);
    if (!target) {
        return target.error();
    }
    scene_target_ = target.value();
    scene_target_width_ = width;
    scene_target_height_ = height;
    return Error::none();
}

Error Renderer3D::ensure_background(rhi::Device& device) {
    if (background_shader_ != nullptr) {
        return Error::none();
    }
    if (background_attempted_) {
        // A previous attempt failed; do not retry every frame.
        return Error("background pass unavailable");
    }
    background_attempted_ = true;

    rhi::ShaderDesc desc;
    desc.name = "renderer3d_background";
    desc.vertex_source = kBackgroundVertexShaderSource;
    desc.fragment_source = kBackgroundFragmentShaderSource;
    auto shader = device.create_shader(desc);
    if (!shader) {
        return shader.error();
    }
    background_shader_ = shader.value();

    auto vertex_buffer = device.create_vertex_buffer(rhi::VertexLayout::position_uv(),
                                                     static_cast<u32>(kBackgroundQuadVertices.size() * sizeof(f32)));
    if (!vertex_buffer) {
        return vertex_buffer.error();
    }
    background_quad_vertex_buffer_ = vertex_buffer.value();
    background_quad_vertex_buffer_->update(std::span<const u8>(
        reinterpret_cast<const u8*>(kBackgroundQuadVertices.data()), kBackgroundQuadVertices.size() * sizeof(f32)));

    auto index_buffer = device.create_index_buffer(static_cast<u32>(kBackgroundQuadIndices.size()));
    if (!index_buffer) {
        return index_buffer.error();
    }
    background_quad_index_buffer_ = index_buffer.value();
    background_quad_index_buffer_->update(std::span<const u8>(
        reinterpret_cast<const u8*>(kBackgroundQuadIndices.data()), kBackgroundQuadIndices.size() * sizeof(u32)));

    return Error::none();
}

void Renderer3D::draw_background(rhi::Device& device, const Camera3D& camera) {
    FE_CHECK(background_shader_ != nullptr);

    // A background fills the frame and must not occlude the geometry that follows,
    // so it draws with no depth write and no depth test, and it does not blend.
    device.set_depth_test(false);
    device.set_blend_mode(rhi::BlendMode::None);
    device.bind_shader(*background_shader_);
    background_shader_->set_vec3("u_top_color", camera.background_top);
    background_shader_->set_vec3("u_bottom_color", camera.background_bottom);
    device.bind_vertex_buffer(*background_quad_vertex_buffer_);
    device.bind_index_buffer(*background_quad_index_buffer_);
    device.draw_indexed(static_cast<u32>(kBackgroundQuadIndices.size()));
    ++background_draws_;
}

Error Renderer3D::ensure_particles(rhi::Device& device) {
    if (particle_shader_ != nullptr) {
        return Error::none();
    }
    if (particle_attempted_) {
        // A previous attempt failed; do not retry every frame.
        return Error("particle pass unavailable");
    }
    particle_attempted_ = true;

    rhi::ShaderDesc desc;
    desc.name = "renderer3d_particles";
    desc.vertex_source = kParticleVertexShaderSource;
    desc.fragment_source = kParticleFragmentShaderSource;
    auto shader = device.create_shader(desc);
    if (!shader) {
        return shader.error();
    }
    particle_shader_ = shader.value();

    auto vertex_buffer = device.create_vertex_buffer(rhi::VertexLayout::sprite_quad(),
                                                     static_cast<u32>(kParticleQuadVertices.size() * sizeof(f32)));
    if (!vertex_buffer) {
        return vertex_buffer.error();
    }
    particle_quad_vertex_buffer_ = vertex_buffer.value();
    particle_quad_vertex_buffer_->update(std::span<const u8>(reinterpret_cast<const u8*>(kParticleQuadVertices.data()),
                                                             kParticleQuadVertices.size() * sizeof(f32)));

    auto index_buffer = device.create_index_buffer(static_cast<u32>(kParticleQuadIndices.size()));
    if (!index_buffer) {
        return index_buffer.error();
    }
    particle_quad_index_buffer_ = index_buffer.value();
    particle_quad_index_buffer_->update(std::span<const u8>(reinterpret_cast<const u8*>(kParticleQuadIndices.data()),
                                                            kParticleQuadIndices.size() * sizeof(u32)));

    return Error::none();
}

void Renderer3D::ensure_particle_capacity(rhi::Device& device, u32 count) {
    // The record uploaded per particle must match the instance layout's stride
    // exactly, or the GPU reads each record shifted by the difference.
    static_assert(sizeof(ParticleRecord) == 2 * 16, "ParticleRecord must match VertexLayout::particle_instance()");

    if (count <= particle_instance_capacity_) {
        return;
    }
    const u32 new_capacity = std::max(count, particle_instance_capacity_ * 2);
    auto buffer = device.create_vertex_buffer(rhi::VertexLayout::particle_instance(),
                                              new_capacity * static_cast<u32>(sizeof(ParticleRecord)));
    if (!buffer) {
        FE_LOG_ERROR("renderer3d: could not grow particle instance buffer: %s", buffer.error().message().c_str());
        return;
    }
    particle_instance_buffer_ = buffer.value();
    particle_instance_capacity_ = new_capacity;
}

void Renderer3D::draw_particles(rhi::Device& device, const Mat4& view_projection, const Camera3D& camera) {
    if (frame_particles_.empty()) {
        return;
    }
    if (Error error = ensure_particles(device); error) {
        FE_LOG_ERROR("renderer3d: particle pass unavailable: %s", error.message().c_str());
        return;
    }

    // Back to front, the same rule as the translucent mesh pass: a billboard
    // blends over what is already in the frame, so the one behind must be drawn
    // first. A stable sort keeps emission order among equal distances, so a burst
    // does not reshuffle frame to frame.
    const Vec3 eye = camera.eye();
    std::stable_sort(frame_particles_.begin(), frame_particles_.end(),
                     [&eye](const render::ParticleInstance& a, const render::ParticleInstance& b) {
                         return (a.position - eye).length_squared() > (b.position - eye).length_squared();
                     });

    ensure_particle_capacity(device, static_cast<u32>(frame_particles_.size()));
    if (particle_instance_buffer_ == nullptr) {
        return;
    }

    frame_particle_records_.clear();
    frame_particle_records_.reserve(frame_particles_.size());
    for (const render::ParticleInstance& particle : frame_particles_) {
        frame_particle_records_.push_back(ParticleRecord{
            Vec4{particle.position.x, particle.position.y, particle.position.z, particle.size}, particle.color});
    }
    particle_instance_buffer_->update(std::span<const u8>(reinterpret_cast<const u8*>(frame_particle_records_.data()),
                                                          frame_particle_records_.size() * sizeof(ParticleRecord)));

    // Transparent state: blend, test depth against the opaque geometry, do not
    // write depth. Restored afterwards so the next frame starts from the opaque
    // default rather than from this pass's state.
    device.set_blend_mode(rhi::BlendMode::Alpha);
    device.set_depth_test(true);
    device.set_depth_write(false);

    const Vec3 up = camera.up();
    Vec3 right = camera.forward().cross(up);
    // A camera looking straight along its own up axis has no defined right; fall
    // back to +X rather than normalising a zero vector into NaNs.
    right = right.length_squared() > 1e-8f ? right.normalized() : Vec3{1.0f, 0.0f, 0.0f};

    device.bind_shader(*particle_shader_);
    particle_shader_->set_mat4("u_view_projection", view_projection);
    particle_shader_->set_vec3("u_camera_right", right);
    particle_shader_->set_vec3("u_camera_up", up);
    device.bind_vertex_buffer(*particle_quad_vertex_buffer_);
    device.bind_instance_buffer(*particle_instance_buffer_);
    device.bind_index_buffer(*particle_quad_index_buffer_);
    device.draw_indexed_instanced(static_cast<u32>(kParticleQuadIndices.size()),
                                  static_cast<u32>(frame_particle_records_.size()));
    last_particles_ = static_cast<u32>(frame_particle_records_.size());

    device.set_blend_mode(rhi::BlendMode::None);
    device.set_depth_write(true);
}

} // namespace fe::renderer3d
