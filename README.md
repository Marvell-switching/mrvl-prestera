# Marvell Prestera Platform specific directories and REPOs

### Kernel Driver Sources (mrvl-prestera)

---

<a href="https://www.marvell.com/"><img src="https://www.marvell.com/content/dam/marvell/en/rebrand/marvell-logo3.svg" alt="Marvell Technologies" width="200"></a>

## Directories and Repositories

| Directory | Description | GitHub Link |
|-----------|-------------|-------------|
| <code>platform/<wbr>marvell-prestera/</code> | Main directory with sai.mk, docker-syncd-mrvl-prestera and submodules | [sonic-net/<wbr>sonic-buildimage](https://github.com/sonic-net/sonic-buildimage.git) |
| <code>platform/<wbr>marvell-prestera/<wbr>mrvl-prestera/</code> | Kernel Driver (.ko) Sources providing Interrupt, DMA, MBUS, Ethernet and legacy DTB files | [Marvell-switching/<wbr>mrvl-prestera](https://github.com/Marvell-switching/mrvl-prestera.git) |
| <code>platform/<wbr>marvell-prestera/<wbr>sonic-platform-marvell/</code> | Platform-device initialisation and monitoring rules | [Marvell-switching/<wbr>sonic-platform-arm64](https://github.com/Marvell-switching/sonic-platform-arm64.git) |
| <code>device/<wbr>marvell/</code> | HWSKU — profiles, ports, ini, XML configurations for boards supported by Marvell | [sonic-net/<wbr>sonic-buildimage](https://github.com/sonic-net/sonic-buildimage.git) |

## Boards / Platforms

| ASIC | SONiC device | CPU | Architecture | Platform |
|------|--------------|-----|---------------------|-----------------|
| AC5X-RD | `arm64-marvell_rd98DX35xx_cn9131-r0` | CN913x | `arm64` | `marvell-prestera` |
| AC5P-RD | `arm64-marvell_rd98DX45xx_cn9131-r0` | CN913x | `arm64` | `marvell-prestera` |
| AC3X | `armhf-nokia_ixs7215_52x-r0` | CN913x | `armhf` | `marvell-prestera` |
| Falcon-12.8T | `x86_64-marvell_db98cx8580_16cd-r0` | Xeon | `amd64` | `marvell-prestera` |
| Falcon-6.4T | `x86_64-marvell_db98cx8580_32cd-r0` | Xeon | `amd64` | `marvell-prestera` |
| Falcon-3.2T | `x86_64-marvell_db98cx8514_10cc-r0` | Xeon | `amd64` | `marvell-prestera` |
| Falcon-2T | `x86_64-marvell_db98cx8522_10cc-r0` | Xeon | `amd64` | `marvell-prestera` |
| Teralynx-10 | `x86_64-marvell_dbmvtx9180-r0` | Xeon D15xx | `amd64` | `marvell-teralynx` |

## Directory Tree

```
platform/marvell-prestera/
mrvl-prestera/
│
├─ debian
│   ├── control
│   ├── mrvlprestera.install.template
│   └── rules
│
├─ drivers
│   ├─ armhf   (Interrupt, MBUS for <armhf> only)
│   │   ├── mvGpioDrv (_Nokia7215-armhf-LED_)
│   │   ├── cpssEnabler/linuxNoKernelModule/drivers/***
│   │   └── common/h/
│   │
│   └─ generic (Int, DMA2, MBUS, Eth for <arm64 and amd64>)
│       ├── cpssEnabler/linuxNoKernelModule/drivers/***
│       └── common/h/
│                          drivers
│                            ├── dmaDriver2.c
│                            ├── dmaDriver.c
│                            ├── ethDriver.c
│                            ├── ethDriver.h
│                            ├── ethOpsDriver.c
│                            ├── intDriver.c
│                            ├── _Makefile
│                            ├── Makefile
│                            ├── manual_make.sh
│                            ├── mbusDriver.c
│                            ├── mbusResources.c
│                            ├── multi-port-init.sh
│                            ├── mvcpss_main.c
│                            ├── mvDriverTemplate.h
│                            ├── mvpci.c
│                            ├── mvResources.h
│                            ├── saiMod.c
│                            └── srcversion.h
├─ platform
    ├─ amd64
    │   ├── common/usr/local/bin/msai
    │   └─ DIRs 98cx8514 , 98cx8522 , 98cx8540 , 98cx8580
    │       ├── etc
    │       │   ├─ modules-load.d/marvell.conf
    │       │   ├─ sonic/eeprom
    │       │   └─ sysctl.d
    │       │       ├── 98-sysctl.conf
    │       │       └── z-sysctl.conf
    │       └── usr/lib/systemd/system-shutdown/marvell
    │
    ├─ arm64
    │   ├── common/usr/local/bin/msai
    │   ├─ DIRs 7020 , 913x , ac5x
    │   │   └── etc
    │   │       ├── fw_env.config
    │   │       ├── modules-load.d/marvell.conf
    │   │       ├── sonic/eeprom
    │   │       └── sysctl.d
    │   │           ├── 98-sysctl.conf
    │   │           └── z-sysctl.conf -> 98-sysctl.conf
    │   └── common/boot/cn9131-db-comexpress.dtb   ------ LEGACY DTB
    │
    └─ armhf
        └─ common
            ├── boot                               ------ LEGACY DTB
            │   ├── armada-385-ET6448M_4G_Nand.dtb
            │   ├── armada-385-ipd6448m-5x.dtb
            │   └── armada-385-ipd6448m.dtb
            ├── etc
            │   ├── fw_env.config
            │   ├── modules-load.d/marvell.conf
            │   ├── sonic/eeprom
            │   └── sysctl.d
            │       ├── 98-sysctl.conf
            │       └── z-sysctl.conf
            └── usr/local/bin/msai
```

The `driver` make-flags and sources are different for `<armhf>`, but same for `<generic>` arm64 and amd64.

The KO module is `mvcpss.ko`; built after `src/sonic-linux-kernel` build and along with `sonic-platform-marvell-DEVICE.deb`. The successfully built `mvcpss.ko` is not seen directly in the whole sonic-buildimage tree, but it is built in to the `sonic-platform-marvell-DEVICE.deb`.

The Kernel is built in DEBIAN tree with SONiC `vermag` properties. So `mvcpss.ko` with the same source code but compiled in a pure Kernel-tree cannot be installed/used in SONiC rootfs.

The DTB files are legacy for 202505 and earlier branches. On 202511 and later branches with Kernel-6.12 the DTS/DTSI are taken from the Kernel Community GitHub and built into FTD/DTB.

## debian/mrvlprestera.install.template influence onto SONiC services startup

During the `mrvlprestera` Debian package build, `debian/rules` expands `debian/mrvlprestera.install.template` (substituting `ARCH`) into `debian/mrvlprestera.install`. That generated file controls which platform files are staged into the SONiC rootfs:

```
platform/ARCH/common/boot/*.dtb /boot
platform/ARCH/common/usr/local/bin/* /usr/local/bin
```

For `amd64`, only the `/usr/local/bin` line is used.

Per-board `platform/<arch>/<board>/etc/` content (for example `modules-load.d/marvell.conf`, `sysctl.d`, `fw_env.config`, and `usr/lib/systemd/system-shutdown/marvell`) is installed from the HWSKU/device overlay and governs early boot: `marvell.conf` loads `eeprom`, `mvcpss`, and `psample` before SONiC container services such as `syncd` can start.

