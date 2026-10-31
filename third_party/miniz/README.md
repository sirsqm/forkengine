# miniz

`miniz.c` / `miniz.h` — an amalgamated DEFLATE (zlib) implementation.

- Version: 11.3.2 (the amalgamated `miniz-3.1.2` release)
- License: MIT (see `LICENSE`)
- Source: https://github.com/richgel999/miniz

It is used by the `pack` module to compress pack entry payloads
(`fe::pack::compress` / `fe::pack::decompress`, see
`docs/adr/0126-pack-compression.md`). A pack is compressed when it is built and
decompressed transparently when it is read, so the codec is an on-disk detail.

Do not edit the files. To update, replace `miniz.c`, `miniz.h`, and `LICENSE`
from the release, then record the new version here and in `../README.md`.
