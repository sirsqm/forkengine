// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/runtime/AudioListenerSystem.h"

#include "fe/audio/AudioSystem.h"
#include "fe/core/Math.h"
#include "fe/renderer2d/Renderer2D.h"
#include "fe/renderer3d/Renderer3D.h"
#include "fe/scene/Camera2D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"

namespace fe {

void AudioListenerSystem::update(const FrameContext& context, Scene& scene) {
    if (audio_ == nullptr) {
        return;
    }

    // A 3D camera is preferred when the scene has one: it is the camera the 3D
    // pass draws through, so it is the one the player hears from. A scene with
    // only a 2D camera falls back to that, which is every 2D game. The two
    // lookups are the renderers' own, so this agrees with what was drawn.
    if (const Camera3D* camera = renderer3d::Renderer3D::find_active_camera(scene); camera != nullptr) {
        // `eye()` is the owner's world position, and already handles a detached
        // camera (returns the origin); a detached camera has no tracked ear, so
        // skip the write rather than move the listener to a position nobody asked
        // for, and drop the velocity so it does not carry stale motion.
        if (const Node* owner = camera->node(); owner != nullptr) {
            const Vec3 eye = camera->eye();
            audio_->set_listener_position(eye);
            set_velocity(context, eye);
            // A 3D camera has an ear direction, so the listener also pans: a
            // source to the player's right is heard in the right ear. This is the
            // half a 2D camera cannot supply - a 2D scene has no up vector, so it
            // gets position only and panning stays off (ADR-0099).
            audio_->set_listener_orientation(camera->forward(), camera->up());
        } else {
            audio_->set_listener_velocity(Vec3{});
            has_previous_position_ = false;
        }
        return;
    }

    // A 2D camera has no `eye()` accessor, but its world position is the same
    // computation: the owner's translation. Reading the world matrix (not the
    // local transform) is what drags the listener with a parented camera.
    //
    // A 2D camera deliberately sets no orientation: panning needs an ear
    // direction and a 2D scene has no up vector, so a 2D game hears a source
    // exactly as it did before panning existed rather than through an invented
    // left/right axis. A stale orientation from an earlier 3D frame is cleared,
    // so switching a scene from 3D to 2D does not leave it panned.
    if (const Camera2D* camera = renderer2d::Renderer2D::find_active_camera(scene); camera != nullptr) {
        if (const Node* owner = camera->node(); owner != nullptr) {
            const Vec3 position = transform_point(owner->world_matrix(), Vec3{});
            audio_->set_listener_position(position);
            set_velocity(context, position);
        }
        audio_->set_listener_orientation(Vec3{}, Vec3{0.0f, 1.0f, 0.0f});
        return;
    }

    // No camera: the listener keeps its position (the update above wrote nothing),
    // but the ear is no longer tracked, so it has no velocity to shift by.
    audio_->set_listener_velocity(Vec3{});
    has_previous_position_ = false;
}

void AudioListenerSystem::set_velocity(const FrameContext& context, const Vec3& position) {
    // The ear's velocity is the position delta over the frame's delta time, the
    // same finite difference the mixer uses for a source's own velocity. The
    // first frame the listener is tracked has no previous sample, so the velocity
    // is zero (no shift); a zero-length delta (a paused step) leaves the previous
    // position in place and reports no velocity, so a pause does not register as
    // a huge jump.
    Vec3 velocity{};
    if (has_previous_position_ && context.delta_time > 0.0f) {
        velocity = (position - previous_position_) / context.delta_time;
    }
    audio_->set_listener_velocity(velocity);
    previous_position_ = position;
    has_previous_position_ = true;
}

} // namespace fe
