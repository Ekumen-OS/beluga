# Beluga VDB

<div align="center">
<img alt="Shows the Beluga logo." src="images/57fe338a-3e7e-46b3-ad2f-17011d6d306a.png" height="200">
</div>

## 🌐 Overview

BelugaVDB is a library extension for `beluga` that integrates [OpenVDB](https://www.openvdb.org/), enabling advanced 3D localization capabilities. Currently, this extension uses OpenVDB to efficiently process 3D maps and pointcloud data. Maps in `vdb` format can be generated using any third party library such as [VDB Mapping](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping).

## 📦 External Dependencies

### OpenVDB

For Ubuntu 24.04 and newer distributions, OpenVDB can be installed using `apt`:

```bash
sudo apt install libopenvdb-dev
```

For older distributions, it must be installed from sources as follows:

#### Step 1: Download OpenVDB

Clone [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb) into your machine (for Jammy version 8.2.0 is recommended):

```bash
git clone -b v8.2.0 https://github.com/AcademySoftwareFoundation/openvdb.git
```

#### Step 2: Compile OpenVDB

Now you need to build OpenVDB:

```bash
cd openvdb && \
mkdir build && cd build && \
cmake .. && \
make -j$(nproc)
```

#### Step 3: Install OpenVDB

You can now install OpenVDB using:

```bash
sudo make install
```

> [!NOTE]
> For more information about OpenVDB please refer to the [official documentation](https://www.openvdb.org/documentation/doxygen/).
