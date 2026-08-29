# Building and sideloading without a Mac

Spec §14. You cannot compile a native iOS app on the iPhone itself, so a build machine is
needed somewhere. The repository is set up for the cheapest route: push, let CI build, download
the `.ipa`.

## The CI route (recommended)

[`.github/workflows/ci.yml`](../.github/workflows/ci.yml) has two jobs.

**`core`** runs on a free Linux runner on every push and pull request. Because
`PerformanceTimerCore` imports nothing from Apple, this is where the entire estimator is
tested — Kalman filter, RTS smoother, UBX parser, crossing solver, the lot. It also fails the
build if an Apple-framework import ever reaches the core, which is the only thing keeping that
property true over time.

**`ipa`** runs on a macOS runner, generates the Xcode project with XcodeGen, builds unsigned
for device, and uploads `PerformanceTimer-unsigned.ipa` as a workflow artifact.

Public repositories get unlimited macOS minutes. Private repositories consume the free
allotment at a **10× multiplier**, which is why the `ipa` job is restricted to `main`, tags,
and manual dispatch rather than running on every push. Trigger it by hand from the Actions tab
when you actually want a build.

[Codemagic](https://codemagic.io) has a free tier of macOS minutes and is purpose-built for
mobile if you would rather not spend GitHub's.

## Getting the .ipa onto the phone

| Tool | Needs | Notes |
|---|---|---|
| **Sideloadly** | Windows or Linux | Simplest if you have no Mac at all |
| **AltStore** | AltServer on a computer on the same network | Refreshes over the network |
| **SideStore** | One-time computer-assisted setup | Then refreshes on-device over a local WireGuard tunnel |

## Signing limits

A free Apple ID gives **7-day certificates**, a 3-app limit, and 10 app IDs per week — so the
app stops launching every week until you refresh it. A paid Developer Program account ($99/yr)
gives 1-year certificates and removes the refresh treadmill.

For a project you will iterate on daily, the paid account pays for itself in avoided
frustration within about a month.

## Renting a Mac

MacinCloud, MacStadium and AWS EC2 Mac instances give full Xcode over VNC. More expensive, but
you get a debugger, Instruments and the Simulator — genuinely worth it for the first few weeks
of a project this sensor-heavy, because the failures you will hit are timing and threading
failures that a log does not show you.

## Generating the project locally

On any Mac with [XcodeGen](https://github.com/yonaskolb/XcodeGen):

```bash
brew install xcodegen
xcodegen generate
open PerformanceTimer.xcodeproj
```

The `.xcodeproj` is generated, not committed — [`project.yml`](../project.yml) is the source of
truth, which keeps the CI build and a local build identical.

## Why not React Native or Expo

Spec §14 is right about this and it is worth restating: Expo EAS Build will produce an iOS
build without a Mac, but pushing 100 Hz CoreMotion samples across a JavaScript bridge
introduces jitter in exactly the signal the filter is most sensitive to. §2 exists because
50–100 ms of timing error is easy to introduce and invisible once introduced; a JS bridge is a
generous source of it.

If you go that way regardless, put the sensor capture and the estimator in a native module and
use JS only for the UI — at which point you may as well write it in Swift.

## Windows development

The core is developed and tested on Windows in this repository. Swift for Windows needs the
MSVC toolchain and the Windows SDK; [`tools/swift.cmd`](../tools/swift.cmd) sets up
`vcvars64` and `SDKROOT`, and [`tools/test.ps1`](../tools/test.ps1) runs the suite and prints a
summary:

```bash
powershell -File tools/test.ps1
```

[`tools/run.cmd`](../tools/run.cmd) runs a built executable with the Swift runtime DLLs on
`PATH` — without it, a built `.exe` exits with `0xC0000135` (DLL not found).

The `App/` directory cannot be compiled on Windows; every file in it is wrapped in
`#if canImport(...)` so the package still builds, and CI compiles it on macOS.
