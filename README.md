# ohrc_pixi_ws

This is as setup file for [OpenHRC](https://github.com/OpenHRC/OpenHRC) on [Pixi](https://pixi.prefix.dev/latest/).

We have tested on
- macOS Tahoe

## 1. Install Pixi
Follow the official instruction [WEB](https://pixi.prefix.dev/latest/installation/)

If you install it on macOS
```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

## 2. Clone this repository
```bash
git clone https://github.com/OpenHRC/ohrc_pixi_ws.git --recursive
```

## 3. Install Dependencies
```bash
cd ohrc_pixi_ws
pixi install
```

### 4. Build OpenHRC (and others in src)
```bash
pixi build
```

