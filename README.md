# SimpleLink™ Low Power F2 and F3 BLE Examples

This repository contains BLE examples for TI devices supported by the
SimpleLink Low Power SDKs (LPF2 and LPF3). To learn which devices are supported
by each SDK, refer to the [SDK Device Association section](#sdk-association).

This repository redistributes examples from TI's [SimpleLink LPF3
SDK](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK) and
[SimpleLink LPF2
SDK](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F2-SDK). These SDKs
are part of the [SimpleLink Low Power
ecosystem](https://www.ti.com/tool/SIMPLELINK-LOWPOWER-SDK).

More details, including supported devices, IDEs, and toolchains are provided in
the respective SDK release notes.

## Repository Layout

The **examples/** directory contains the same examples provided in those
SDKs, in the same directory structure.

The LPF2 and LPF3 SDKs are provided as [Git
submodules](https://www.git-scm.com/docs/gitsubmodules) in their respective
subdirectories.  If you're only interested in devices supported by the LPF3 SDK,
you will only need to initialize and update the LPF3 SDK submodule.

As a quick reference, you can initialize and update a single Git submodule in
one step like this:

```bash
# To initialize and update the F2 SDK
.../simplelink-ble5stack-examples$ git submodule update --init cc13xx_cc26xx_sdk

# To initialize and update the F3 SDK
.../simplelink-ble5stack-examples$ git submodule update --init simplelink-lowpower-f3-sdk
```

Alternatively you can initialize/update _all_ submodules when cloning a repo
with `git clone --recurse-submodules {repo-ref}`.  See Git documentation for
details.

Once initialized and updated, you can refer to each SDK's README.md and Release
Notes for details on how to download its dependencies, and build its libraries.

* [SimpleLink LPF2 SDK
  README](https://github.com/TexasInstruments/cc13xx_cc26xx_sdk/blob/main/README.md)
* [SimpleLink LPF3 SDK
  README](https://github.com/TexasInstruments/simplelink-lowpower-f3-sdk/blob/main/README.md)

> Note, the links above are to online copies of the latest SDK READMEs.  They
> are useful for online readers, but be sure to consult the SDK submodule's
> _actual_ README.md after cloning, checking out your branch/tag, and updating
> your submodule, as details may change from release to release.

## Setup Instructions

### Build SDK Libraries

Each time you update an SDK submodule, you will need to build its libraries.
This process _can_ vary between the different SDKs, so refer to each SDK's
README.md for specifics, but generally you will need to edit the **imports.mak**
file at the top, then run `make`.

Note that _sometimes_ the dependencies can vary from SDK to SDK.  For example,
if you've been using the F2 SDK and SysConfig version X, and want to start using
the F3 SDK, it may require a newer SysConfig version.  So, again, be sure to
refer to each SDK's README.md and Release Notes.

Often newer versions of dependencies are compatible, so you can use
newer-and-compatible versions than the SDK was validated against.  But each SDK
does have its own **imports.mak** so you _can_ specify different dependency
versions for each SDK if needed.

## Build Examples

After building the SDK libraries, you can build the BLE examples.  The
examples support a few ways to build:

* [Command line makefile](#build-examples-from-command-line)
* [CCS IDE](#build-examples-from-ccs)
* [IAR IDE](#build-examples-from-iar)

### Build Examples From Command Line

Remember, before building the examples, you must build the SDK libraries!

To build an example from the command line using [GNU
make](https://www.gnu.org/software/make/manual/make.html), change into the
appropriate example's directory (e.g.
**{rtos}/{board}/ble5stack/{example}/{rtos}/{toolchain}**), then run `make`.

```bash
.../simplelink-ble5stack-examples$ cd examples/rtos/LP_CC2651R3SIPA/ble5stack/basic_ble/freertos/ticlang/
.../ticlang$ make
```

Note, you can also clean the example with `make clean`.

### Build Examples From CCS

Remember, before building the examples, you must build the SDK libraries!

The examples also include TI Code Composer Studio (CCS) project support,
enabling them to be imported into, and built by, CCS.

Before importing the example, the SDK(s) location must be registered with CCS:

1. Preferences->Code Composer Studio->Products
2. Select Add...
3. Navigate to the SDK submodule location
4. Select Open

Repeat for each SDK you will be using.  This registers the SDK with CCS.
Successful registration of an SDK will show it in the "Discovered
Products" list:

![CCS Add Products Dialog](images/add_products.png)

Now you can import an example!

1. Project->Import CCS Project...
2. Select search-directory->Browse...
3. Navigate to a directory within your clone of the example repo to search for
   examples and Select Folder
4. Select the example(s) you wish to import and press Finish

![Import CCS Projects Dialog](images/select_ccsproject.png)

### Build Examples from IAR

Remember, before building the examples, you must build the SDK libraries!

Follow the instructions in your respective SDK's Quick Start Guide:

* [SimpleLink LPF2 SDK Quick Start Guide](https://dev.ti.com/tirex/explore/node?node=A__AC7UNBWx3i6iMAUzzhqKwA__com.ti.SIMPLELINK_CC13XX_CC26XX_SDK__BSEc4rl__LATEST)
* [SimpleLink LPF3 SDK Quick Start Guide](https://dev.ti.com/tirex/explore/node?node=A__AC7UNBWx3i6iMAUzzhqKwA__com.ti.SIMPLELINK_LOWPOWER_F3_SDK__58mgN04__LATEST)

## SDK Association

Click the links below for a list of boards in each SDK that BLE examples
are provided for. Note that TI does not provide boards for every device, so in
some cases you may need to SysConfig-migrate an example from a similar board to
your specific device.

* [SimpleLink LPF2 boards](images/simplelink_cc13xx_cc26xx_sdk.md)
* [SimpleLink LPF3 boards](images/simplelink_lowpower_f3_sdk.md)
