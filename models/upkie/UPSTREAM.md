# Upkie baseline model provenance

This directory vendors the Upkie variant from
[`MarcDcls/mjlab_upkie`](https://github.com/MarcDcls/mjlab_upkie).

- Upstream commit: `d7895789a601943f78b3b9bcdc4def7b444f9913`
- Retrieved: 2026-09-16
- Upstream path: `src/mjlab_upkie/robot/upkie/`
- License: Apache-2.0; see `LICENSE`
- Local changes to `upstream/`: none

The model is Marc Duclusaud's physical Upkie variant rather than an official
MJCF published by the Upkie project. Its nominal model mass is 5.4605 kg.
The official Upkie hardware and software repositories are:

- <https://github.com/upkie/upkie>
- <https://github.com/upkie/upkie_description>

Future mechanical variants must live outside `upstream/` so this baseline
remains byte-for-byte comparable with the pinned source.
