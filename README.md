# mple

Motion Planning Library for EPICS

> [!WARNING]
> This is a work in progress. Expect breaking changes at any time.

## Building

`mple` depends on [mple-support](https://github.com/nmarks99/mple-support) which must be built first.
Once you have successfully built `mple-support` on your system, clone and build `mple`:

``` bash
# Clone with submodules
git clone --recurse-submodules https://github.com/nmarks99/mple.git
# ...or if already cloned without submodules, run:
# git submodule update --init --recursive

# Edit mple/configure/CONFIG_SITE.local to point to mple-support, then
make
```
