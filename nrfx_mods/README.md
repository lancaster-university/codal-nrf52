# nrfx Mods

This directory contains copies of nrfx source files, with needed modifications and explanations of what they are.

nrfx is a submodule of this repository, but where applicable, the CMake file checks for files in this directory first. 

## Updating nrfx

Should the nrfx library ever be updated, any updates to the original files duplicated here should also be applied to these files.

Each modified file here *should* contain information about the changes applied.

Currently the https://github.com/microbit-foundation/codal-microbit-nrf5sdk/ library depends on this nrfx submodule.
Because the nrf5SDK is no longer updated by Nordic, it has a hard dependency on nrfx being at v2.
nrfx v3 introduces breaking changes that will likley impact the nRFSDK, so we might be stuck on v2.
