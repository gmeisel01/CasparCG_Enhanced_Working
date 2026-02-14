# CasparCG v2.5 with Audio-Video Sync & new Port Audio Multi-Channel ASIO Consumer

## What This PR Does
1. **Implements video-scheduled audio dispatch** to address audio-video sync drift issues
2. **Adds PortAudio consumer** for professional multi-channel ASIO output
3. **Enhances screen consumer** with flexible output configuration for professional AV workflows

## Key Features
- **Video-scheduled dispatch**: Audio packets tied to precise video frame timestamps
- **Auto-tuning**: Statistical analysis automatically adjusts latency compensation
- **Frame-accurate architecture**: Audio intrinsically synchronized to video timing
- **ASIO support**: Native multi-channel audio for professional interfaces
- **Custom pixel space**: Arbitrary aspect ratios and resolutions for presentation systems

## Configuration
```xml
<system-audio>
  <latency-compensation-ms>40</latency-compensation-ms>
  <auto-tune-latency>true</auto-tune-latency>
</system-audio>

<portaudio>
  <device-name>Your ASIO Device</device-name>
  <output-channels>8</output-channels>
  <latency-compensation-ms>40</latency-compensation-ms>
  <auto-tune-latency>true</auto-tune-latency>
</portaudio>

<screen>
  <aspect-ratio>3840/1080</aspect-ratio>  <!-- Custom pixel space for presentation systems -->
  <brightness-boost>1.2</brightness-boost>
  <saturation-boost>1.1</saturation-boost>
</screen>
```

## Implementation Status
- ✅ **Implemented**: Video-scheduled audio dispatch for OpenAL and PortAudio
- ✅ **Implemented**: Auto-tuning system with statistical analysis
- ✅ **Implemented**: Custom display configuration merged with 2.5 codebase
- ✅ **Builds successfully**: Windows (MSVC)
- ⚠️ **Not yet tested**: Formal benchmarking with measurements
- ⚠️ **Linux**: CI infrastructure issue (not code-related)

## Breaking Changes
None. All changes are additive or internal improvements.

---

<details>
<summary><b>📖 Full Technical Details (Click to Expand)</b></summary>

# Audio-Video Synchronization & Multi-Channel Audio Support

## Problem Statement
CasparCG 2.5's audio consumer can exhibit audio-video synchronization drift during extended playback, particularly in complex processing pipelines. Audio and video subsystems operate independently, which can lead to progressive drift under certain conditions.

## Solution: Video-Scheduled Audio

### Architecture
This implementation introduces a video-scheduled audio dispatch system where audio packets are scheduled for delivery at precise moments based on video frame timing, rather than playing independently.

**Core Components:**
- **Frame-Accurate Dispatch**: Audio packets scheduled to video frame timestamps
- **Real-Time Thread**: High-priority dispatch thread (THREAD_PRIORITY_TIME_CRITICAL)
- **Auto-Tuning**: Statistical analysis automatically adjusts latency compensation
- **Robust Buffer Management**: Proper OpenAL buffer lifecycle prevents "invalid operation" errors

### How It Works
```cpp
// Calculate precise target time based on video frame timing
auto frame_time_offset = std::chrono::duration<double>(frame_number * frame_duration_seconds_);
auto latency_compensation = std::chrono::milliseconds(audio_latency_compensation_ms_.load());
packet.target_time = channel_start_time_ + frame_time_offset - latency_compensation;

// High-priority thread dispatches at exact moment
std::this_thread::sleep_until(packet.target_time);
dispatch_audio_packet(packet);
```

### Auto-Tuning
The system monitors timing accuracy over 500+ samples and automatically adjusts latency compensation when consistent drift is detected:

```cpp
// Conservative adjustment only with consistent timing (stddev < 20ms)
if (std::abs(avg_error_ms) >= 3 && stddev_ms < 20) {
    int adjustment = static_cast<int>(avg_error_ms * 0.75);
    audio_latency_compensation_ms_ -= adjustment;
}
```

---

## PortAudio Consumer: Multi-Channel ASIO Support

### Features
- **ASIO Native**: Direct ASIO output for professional audio interfaces
- **Video-Scheduled**: Same timing architecture as OpenAL consumer
- **Lock-Free FIFO**: Efficient buffering between dispatch and callback threads
- **Channel Flexibility**: Supports 2-32+ output channels

### Use Cases
- Multi-channel broadcast installations
- Professional audio routing to external mixers
- ASIO hardware integration (RME, MOTU, Universal Audio, etc.)
- Low-latency monitoring workflows

### Configuration Example
```xml
<portaudio>
  <device-name>ASIO Device Name</device-name>
  <output-channels>8</output-channels>
  <latency-compensation-ms>40</latency-compensation-ms>
  <auto-tune-latency>true</auto-tune-latency>
  <buffer-size-frames>128</buffer-size-frames>
  <fifo-ms>50</fifo-ms>
</portaudio>
```

---

## Screen Consumer Enhancements

### Flexible Output Configuration for Professional AV Workflows

Enhanced the 2.5 screen consumer to support custom pixel space configurations for integration with professional presentation systems (Barco E2, Spyder X80, Analog Way systems, etc.).

**Custom Aspect Ratios and Resolutions**
```xml
<aspect-ratio>3840/1080</aspect-ratio>  <!-- Division format for ultra-wide -->
<aspect-ratio>3.555</aspect-ratio>      <!-- Decimal format -->
```

Supports arbitrary aspect ratios and pixel dimensions for:
- Ultra-wide display configurations
- Custom presentation system inputs
- Multi-projector installations
- Video wall processors
- Any non-standard output format

**Visual Calibration Controls**
```xml
<brightness-boost>1.2</brightness-boost>  <!-- Output brightness adjustment -->
<saturation-boost>1.1</saturation-boost>  <!-- Color saturation control -->
```

Shader-based controls for:
- Ambient light compensation
- Display calibration
- Color enhancement for presentation environments

**Multi-Display Spanning**
```xml
<x>0</x>
<y>-1080</y>           <!-- Extended desktop positioning -->
<width>7680</width>    <!-- Custom horizontal resolution -->
<height>2160</height>  <!-- Custom vertical resolution -->
```

Support for:
- Extended desktop configurations
- Multi-display spanning (NVIDIA Mosaic, AMD Eyefinity, etc.)
- Presentation system input requirements
- Custom pixel space perimeters

**Enhanced Display Enumeration** (Windows)
- Comprehensive multi-display detection with detailed logging
- Explicit device selection via `screen-index`
- Primary display fallback with warnings
- Full device property reporting

### Integration Workflow

## Building from Source

### Prerequisites

**Required:**
- Windows 10/11 (64-bit)
- CMake 3.24 or later
- Visual Studio 2022 (Community Edition or higher)
- Git

**Recommended:**
- vcpkg (for dependency management)
- NVIDIA Quadro or similar professional GPU (for best performance)

### Build Instructions

#### 1. Clone the Repository

```bash
git clone https://github.com/gmeisel01/CasparCG_Enhanced_Working.git
cd CasparCG_Enhanced_Working
```

#### 2. Install Dependencies with vcpkg

```bash
# Clone vcpkg if you don't have it
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Install CasparCG dependencies
.\vcpkg install --triplet x64-windows
```

#### 3. Configure with CMake

```bash
# From the CasparCG_Enhanced_Working root directory
mkdir build
cd build

# Configure (replace path to vcpkg as needed)
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -A x64

# Or if vcpkg is in parent directory:
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../vcpkg/scripts/buildsystems/vcpkg.cmake -A x64
```

#### 4. Build

```bash
# Build Release configuration
cmake --build . --config Release

# Or build Debug for development
cmake --build . --config Debug
```

#### 5. Run

```bash
cd Release  # or Debug
.\casparcg.exe
```

### Build Notes

- **PortAudio**: The build automatically includes PortAudio dependencies (DLL, lib, headers in `dependencies/portaudio/`)
- **First build**: May take 20-40 minutes depending on your system
- **Incremental builds**: Much faster (1-5 minutes)
- **Output location**: Executable is in `build/Release/` or `build/Debug/`

### Troubleshooting

**CMake can't find dependencies:**
```bash
# Make sure vcpkg toolchain path is correct
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/full/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -A x64
```

**Build errors with PortAudio:**
- PortAudio binaries are included in `dependencies/portaudio/`
- Ensure this folder exists in your clone
- Headers should be in `dependencies/portaudio/include/portaudio.h`

**Linker errors:**
- Try cleaning: `cmake --build . --config Release --clean-first`
- Or delete `build/` folder and reconfigure from scratch

---

## Configuration

### Audio-Video Sync (OpenAL)

Edit `casparcg.config`:

```xml
<system-audio>
  <latency-compensation-ms>40</latency-compensation-ms>
  <auto-tune-latency>true</auto-tune-latency>
</system-audio>
```

**Settings:**
- `latency-compensation-ms`: Initial latency offset (default: 40ms)
- `auto-tune-latency`: Enable automatic adjustment (default: false)

### Multi-Channel ASIO (PortAudio)

```xml
<portaudio>
  <device-name>Your ASIO Device Name</device-name>
  <output-channels>8</output-channels>
  <latency-compensation-ms>40</latency-compensation-ms>
  <auto-tune-latency>true</auto-tune-latency>
  <buffer-size-frames>128</buffer-size-frames>
  <fifo-ms>50</fifo-ms>
</portaudio>
```

**Settings:**
- `device-name`: ASIO device (leave empty for default)
- `output-channels`: Number of output channels (2-32+)
- `buffer-size-frames`: ASIO buffer size (64, 128, 256, 512)
- `fifo-ms`: Internal buffer size in milliseconds (default: 50ms)

**ASIO Setup:**
1. Install ASIO driver for your audio interface (or ASIO4ALL/FlexASIO)
2. Configure ASIO control panel to match CasparCG sample rate (typically 48000 Hz)
3. Set buffer size in ASIO control panel
4. Test with 2 channels first before using multi-channel

### Custom Display Configuration (Screen Consumer)

```xml
<screen>
  <device>0</device>
  <aspect-ratio>3840/1080</aspect-ratio>
  <stretch>fill</stretch>
  <windowed>false</windowed>
  <x>0</x>
  <y>0</y>
  <width>7680</width>
  <height>2160</height>
  <brightness-boost>1.2</brightness-boost>
  <saturation-boost>1.1</saturation-boost>
  <always-on-top>false</always-on-top>
  <vsync>false</vsync>
</screen>
```

**Custom Aspect Ratios:**
- Division format: `<aspect-ratio>3840/1080</aspect-ratio>`
- Decimal format: `<aspect-ratio>3.555</aspect-ratio>`
- Supports any ratio for presentation systems

**Multi-Display Spanning:**
- Set `x`, `y` for extended desktop positioning
- Set `width`, `height` for custom resolutions
- Works with NVIDIA Mosaic, AMD Eyefinity, or Windows extended desktop

**Visual Calibration:**
- `brightness-boost`: 0.1 to 2.0 (1.0 = no change)
- `saturation-boost`: 0.1 to 2.0 (1.0 = no change)
- Applied in shader for GPU-accelerated processing

The screen consumer provides flexible output that feeds downstream presentation systems, which handle final display routing and processing.

### Compatibility Note
The screen consumer enhancements required merging features from a custom implementation with the current 2.5 codebase. All 2.5 features have been preserved:
- GPU texture strategy for hardware sources (DeckLink/NDI)
- High bit-depth support (10-bit/16-bit channels)
- Display strategy pattern architecture

The merged implementation maintains full 2.5 compatibility while adding flexible output configuration for professional AV workflows.

---

## Technical Implementation Details

### Thread-Safe Audio Scheduling
```cpp
std::priority_queue<audio_packet> audio_schedule_;
std::mutex schedule_mutex_;
std::condition_variable schedule_cv_;
std::thread audio_dispatch_thread_;
```

### Shader Integration
Pre-built shader headers bypass bin2c() narrowing conversion issues on MSVC with `/WX`. Headers use raw string literals for maintainability:

```cpp
static const char* fragment_shader = R"shader(
    #version 450
    uniform float brightness_boost;
    uniform float saturation_boost;
    // ... GLSL code with RGB color space processing ...
)shader";
```

Brightness and saturation controls applied in shader (RGB color space only, not DataVideo YUV modes) for efficient GPU-based adjustment.

---

## Development Environment

**Developed for:**
- Professional broadcast and presentation workflows
- Integration with presentation switchers (Barco, Analog Way, etc.)
- Multi-channel audio routing
- Custom resolution display systems

**Platform:**
- Windows with Visual Studio 2022/2026
- NVIDIA Quadro GPU hardware
- Blackmagic DeckLink interfaces
- Custom resolution outputs (tested up to 7680x2160)

**Target platforms:**
- Windows (tested, working)
- Linux (CI infrastructure issue, not code-related)
- macOS (not yet tested)

---

## Files Changed

### Audio Consumers
- `modules/oal/consumer/oal_consumer.cpp` - Video-scheduled OpenAL implementation
- `modules/oal/consumer/oal_consumer.h` - Interface updates
- `modules/portaudio/portaudio_consumer.cpp` - New ASIO consumer
- `modules/portaudio/portaudio_consumer.h` - PortAudio consumer interface

### Screen Consumer
- `modules/screen/consumer/screen_consumer.cpp` - Merged 2.5 + custom output features
- `modules/screen/CMakeLists.txt` - Pre-built shader headers (bin2c disabled)
- `modules/screen/consumer_screen_fragment.h` - Fragment shader with calibration uniforms
- `modules/screen/consumer_screen_vertex.h` - Vertex shader

### Build System
- Root `CMakeLists.txt` - PortAudio integration (if applicable)

---

## Configuration Migration
Existing `<system-audio>` and `<screen>` configurations continue working unchanged. New features are opt-in:
- `auto-tune-latency` defaults to `false`
- PortAudio consumer requires explicit configuration
- Custom aspect ratios default to 16:9 if not specified
- Brightness/saturation default to 1.0 (no adjustment)

---

## Future Work
- Formal benchmarking with measured performance data
- Cross-platform testing and validation (Linux/macOS)
- Additional statistical metrics for tuning quality assessment
- Configuration presets for common presentation system workflows
- Integration with other consumers for system-wide sync

---

## Acknowledgments
Video-scheduled audio architecture adapted from techniques used in professional NLE systems. Addresses sync issues documented in CasparCG GitHub issues #836, #155, and community reports.

Implementation developed for professional broadcast and presentation workflows requiring frame-accurate audio-video synchronization and flexible output configuration.

---

## Build Status
✅ **Windows**: Builds successfully with MSVC  
⚠️ **Linux**: CI infrastructure issue in base repository (exit code 126), not related to code changes  
❓ **Performance Testing**: Formal benchmarking not yet conducted

</details>
