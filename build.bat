@echo off
setlocal
chcp 65001 >nul
set "ROOT=%~dp0"

set "VSDEV=%ProgramFiles%\Microsoft Visual Studio\18\Insiders\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" set "VSDEV=%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" set "VSDEV=%ProgramFiles%\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" set "VSDEV=%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" set "VSDEV=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" for /f "usebackq delims=" %%I in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDEV=%%I\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" if exist "%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe" for /f "usebackq delims=" %%I in (`"%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDEV=%%I\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" (
  echo VsDevCmd.bat not found.
  exit /b 1
)

call "%VSDEV%" -arch=x64 -host_arch=x64
if errorlevel 1 exit /b 1
where cl >nul 2>nul
if errorlevel 1 (
  echo cl.exe not found after loading Visual Studio environment.
  exit /b 1
)

pushd "%ROOT%"
if errorlevel 1 exit /b 1
if not exist "bin\Release" mkdir "bin\Release"
if not exist "bin\obj\dtsx-decode" mkdir "bin\obj\dtsx-decode"
if not exist "bin\obj\dcadec" mkdir "bin\obj\dcadec"
del /q "bin\obj\dtsx-decode\*.obj" "bin\obj\dcadec\*.obj" 2>nul

for %%F in (
  "bin\win\x64\DTSXDecoder.dll"
  "bin\win\x64\MSVCP140_APP.dll"
  "bin\win\x64\VCCORLIB140_APP.dll"
  "bin\win\x64\VCRUNTIME140_1_APP.dll"
  "bin\win\x64\VCRUNTIME140_APP.dll"
) do if not exist "%%~F" (
  echo P2 runtime resource not found: %%~F
  exit /b 1
)

rc /nologo /fo "bin\obj\dtsx-decode\p2_resources.res" ^
  src\io\p2_resources.rc
if errorlevel 1 exit /b 1

cl /nologo /O2 /W0 /TC /D_CRT_SECURE_NO_WARNINGS ^
  /D_USE_MATH_DEFINES /DNDEBUG /Ithird_party\dcadec ^
  third_party\dcadec\bitstream.c third_party\dcadec\core_decoder.c ^
  third_party\dcadec\dca_context.c third_party\dcadec\dmix_tables.c ^
  third_party\dcadec\exss_parser.c third_party\dcadec\idct_fixed.c ^
  third_party\dcadec\idct_float.c third_party\dcadec\interpolator.c ^
  third_party\dcadec\interpolator_fixed.c ^
  third_party\dcadec\interpolator_float.c ^
  third_party\dcadec\lbr_decoder.c third_party\dcadec\ta.c ^
  third_party\dcadec\xll_decoder.c /c /Fo:"bin\obj\dcadec\\"
if errorlevel 1 exit /b 1

cl /nologo /std:c++17 /utf-8 /EHsc /O2 /W4 ^
  /D_CRT_SECURE_NO_WARNINGS /D_USE_MATH_DEFINES /DNDEBUG ^
  /wd4146 /I. /Isrc /Ithird_party\dcadec ^
  src\app\main.cpp src\app\object_frame_decoder.cpp src\app\options.cpp ^
  src\app\pipeline.cpp src\audio\dca_bed_decoder.cpp ^
  src\audio\layout.cpp ^
  src\bitstream\dtsx_bitstream.cpp src\bitstream\dtsx_segment.cpp ^
  src\bitstream\dtsx_word_buffer.cpp src\dtsx\crc16.cpp ^
  src\dtsx\core_substream.cpp src\dtsx\descriptor_metadata.cpp ^
  src\dtsx\exss_header.cpp src\dtsx\exss_asset.cpp ^
  src\dtsx\frame_assembler.cpp src\dtsx\frame_header.cpp ^
  src\dtsx\frame_sync.cpp src\dtsx\uhd_frame.cpp ^
  src\dtsx\lbr_chunk.cpp src\dtsx\raw_info.cpp ^
  src\dtsx\metadata_chunk.cpp src\dtsx\object_metadata_block.cpp ^
  src\dtsx\object_metadata_header.cpp src\dtsx\object_metadata_updates.cpp ^
  src\dtsx\object_coordinates.cpp src\dtsx\object_spatial_metadata.cpp ^
  src\dtsx\object_waveform_map.cpp src\dtsx\preliminary_metadata.cpp ^
  src\dtsx\speaker_mask.cpp src\dtsx\xll_channel_set.cpp ^
  src\dtsx\xll_channel_parameters.cpp src\dtsx\xll_channel_decoder.cpp ^
  src\dtsx\xll_common_header.cpp src\dtsx\xll_entropy.cpp ^
  src\dtsx\xll_frame_decoder.cpp src\dtsx\xll_navigation.cpp ^
  src\dtsx\xll_prediction.cpp src\io\dts_frame_reader.cpp ^
  src\io\ffmpeg.cpp src\io\object_sidecar_writer.cpp ^
  src\io\object_stem_writer.cpp src\io\process.cpp src\io\wav_writer.cpp ^
  src\io\p2_decoder.cpp ^
  src\render\gain_interpolator.cpp src\render\layout_panner.cpp ^
  src\render\parma_layout.cpp src\render\parma_blind_config.cpp ^
  src\render\parma_pairwise.cpp src\render\parma_pairwise_analysis.cpp ^
  src\render\parma_critical_bands.cpp src\render\parma_filterbank.cpp ^
  src\render\parma_blind_renderer.cpp ^
  src\render\object_gain.cpp src\render\object_audio_renderer.cpp ^
  src\render\object_mixer.cpp src\render\vector_base_panner.cpp ^
  /c /Fo:"bin\obj\dtsx-decode\\"
if errorlevel 1 exit /b 1

cl /nologo ^
  bin\obj\dtsx-decode\main.obj ^
  bin\obj\dtsx-decode\object_frame_decoder.obj ^
  bin\obj\dtsx-decode\options.obj ^
  bin\obj\dtsx-decode\pipeline.obj ^
  bin\obj\dtsx-decode\dca_bed_decoder.obj ^
  bin\obj\dtsx-decode\layout.obj ^
  bin\obj\dtsx-decode\dtsx_bitstream.obj ^
  bin\obj\dtsx-decode\dtsx_segment.obj ^
  bin\obj\dtsx-decode\dtsx_word_buffer.obj ^
  bin\obj\dtsx-decode\crc16.obj ^
  bin\obj\dtsx-decode\core_substream.obj ^
  bin\obj\dtsx-decode\descriptor_metadata.obj ^
  bin\obj\dtsx-decode\exss_header.obj ^
  bin\obj\dtsx-decode\exss_asset.obj ^
  bin\obj\dtsx-decode\frame_assembler.obj ^
  bin\obj\dtsx-decode\frame_header.obj ^
  bin\obj\dtsx-decode\frame_sync.obj ^
  bin\obj\dtsx-decode\uhd_frame.obj ^
  bin\obj\dtsx-decode\lbr_chunk.obj ^
  bin\obj\dtsx-decode\raw_info.obj ^
  bin\obj\dtsx-decode\metadata_chunk.obj ^
  bin\obj\dtsx-decode\object_metadata_block.obj ^
  bin\obj\dtsx-decode\object_metadata_header.obj ^
  bin\obj\dtsx-decode\object_metadata_updates.obj ^
  bin\obj\dtsx-decode\object_coordinates.obj ^
  bin\obj\dtsx-decode\object_spatial_metadata.obj ^
  bin\obj\dtsx-decode\object_waveform_map.obj ^
  bin\obj\dtsx-decode\preliminary_metadata.obj ^
  bin\obj\dtsx-decode\speaker_mask.obj ^
  bin\obj\dtsx-decode\xll_channel_set.obj ^
  bin\obj\dtsx-decode\xll_channel_parameters.obj ^
  bin\obj\dtsx-decode\xll_channel_decoder.obj ^
  bin\obj\dtsx-decode\xll_common_header.obj ^
  bin\obj\dtsx-decode\xll_entropy.obj ^
  bin\obj\dtsx-decode\xll_frame_decoder.obj ^
  bin\obj\dtsx-decode\xll_navigation.obj ^
  bin\obj\dtsx-decode\xll_prediction.obj ^
  bin\obj\dtsx-decode\dts_frame_reader.obj ^
  bin\obj\dtsx-decode\ffmpeg.obj ^
  bin\obj\dtsx-decode\object_sidecar_writer.obj ^
  bin\obj\dtsx-decode\object_stem_writer.obj ^
  bin\obj\dtsx-decode\process.obj ^
  bin\obj\dtsx-decode\wav_writer.obj ^
  bin\obj\dtsx-decode\p2_decoder.obj ^
  bin\obj\dtsx-decode\gain_interpolator.obj ^
  bin\obj\dtsx-decode\layout_panner.obj ^
  bin\obj\dtsx-decode\parma_layout.obj ^
  bin\obj\dtsx-decode\parma_blind_config.obj ^
  bin\obj\dtsx-decode\parma_pairwise.obj ^
  bin\obj\dtsx-decode\parma_pairwise_analysis.obj ^
  bin\obj\dtsx-decode\parma_critical_bands.obj ^
  bin\obj\dtsx-decode\parma_filterbank.obj ^
  bin\obj\dtsx-decode\parma_blind_renderer.obj ^
  bin\obj\dtsx-decode\object_gain.obj ^
  bin\obj\dtsx-decode\object_audio_renderer.obj ^
  bin\obj\dtsx-decode\object_mixer.obj ^
  bin\obj\dtsx-decode\vector_base_panner.obj ^
  bin\obj\dcadec\bitstream.obj bin\obj\dcadec\core_decoder.obj ^
  bin\obj\dcadec\dca_context.obj bin\obj\dcadec\dmix_tables.obj ^
  bin\obj\dcadec\exss_parser.obj bin\obj\dcadec\idct_fixed.obj ^
  bin\obj\dcadec\idct_float.obj bin\obj\dcadec\interpolator.obj ^
  bin\obj\dcadec\interpolator_fixed.obj ^
  bin\obj\dcadec\interpolator_float.obj ^
  bin\obj\dcadec\lbr_decoder.obj bin\obj\dcadec\ta.obj ^
  bin\obj\dcadec\xll_decoder.obj ^
  bin\obj\dtsx-decode\p2_resources.res ^
  /Fe:"bin\Release\dtsx-decode.exe"
if errorlevel 1 exit /b 1

echo Build: bin\Release\dtsx-decode.exe
popd
exit /b 0
