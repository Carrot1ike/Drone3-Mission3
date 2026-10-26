# 카메라 픽셀 보존과 실제 전송의 구분

2026-10-09 확인. 기존 실시간 제어기는 렌더가 만든 native mono8 320×320 배열을 직접 받았다. 보관용 `camera.avi`만 MJPEG로 인코딩했다. 디코딩 후 세 색 채널이 같고 해상도가320×320인 것은 회색 영상/해상도 증거이며, 원본 픽셀 동일성의 증거는 아니다.

첫 entry-hold421의 phase PNG와 AVI 첫 프레임은 최대15 gray-level, 평균 절대차.617715였다. 실제 프레임·난수·패턴3장 round-trip에서도 MJPG는 모두 달랐고 FFV1은 세 장 byte 동일이었다. 근거는 `artifacts/mission3/acquisition/20261009-video-serialization-check/summary.json`이다. 이전 대체 디코더와 homography 결과는 손실 압축 재생으로 재분류하며 immutable plan/result는 수정하지 않는다. 당시 live 목표ID0회는 압축 전 입력의 관측 로그이므로 유효하다.

`scripts/mission3/recording.py`는 native shape/dtype를 검사하고 FFV1을 열며, 입력 byte의 SHA256을 `camera_frames.jsonl`에 프레임마다 남긴다. `scripts/validate_mission3_camera_recording.py`는 모든 디코딩 프레임의 해상도·mono 채널·framecount·source SHA256을 검사한다. 원본 픽셀을 확대하거나 변환하지 않는다. codec unavailable는 오류이며 MJPEG로 몰래 fallback하지 않는다.

실제 전체 녹화 검증 결과: `20261009-lossless-entryhold-entry1-seed421`5959장, `20261009-lossless-coast-v3-entry1-seed439`5041장, 총11000장 모두 일치/누락hash0/다른pixel0. coast439는 이전 같은 조건의 commands/truth/observations/motors/rpyt_packets/deck_samples/estimates7개 파일도 byte 동일했다. 보관 변경 때문에 비행이 달라졌다는 증거는 없다. entry-hold 최초 실행은 인계 slew 수정 이전이어서 그 원실행과 전체 비행의 numeric equivalence를 주장하지 않는다. 각각 접촉 실패로 끝났고 재사용 training이며 새 holdout이 아니다.

무손실421의 six-preset benchmark도 목표ID0회다. 작은 threshold C0에서 잘못된ID57회가 생겼다(이전 MJPEG는15회). 파일 위치는 `artifacts/mission3/acquisition/20261009-lossless-native-detector-bench-v2/`다. 영상 압축이 후보/오검출 수를 바꿀 수 있음을 확인했으며 실패 원인을 문턱 하나로 해결했다고 하지 않는다.

FFV1은 연구 데이터 보관 선택S이고 AI-deck 센서나 Wi-Fi 전송 코덱P가 아니다. 아래 공식 소프트웨어는 RAW/JPEG를 지원하지만, 실기에서 실제 선택한 해상도/전송모드/노출/프레임률/지연/손실을 확인해야 한다.

## 공식 AI-deck 소프트웨어를 읽고 확인한 경계

2026-10-09 읽은 [Bitcraze wifi streamer 소스 d39d2ec](https://github.com/bitcraze/aideck-gap8-examples/tree/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/wifi-img-streamer)는 제조사 제공 **소프트웨어C**다. local read-only snapshot과 각 source SHA256은 `docs/references/bitcraze-aideck-streaming-20261009/source-manifest.json`에 있다. [공식 예제 문서](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/examples/wifi-streamer/)의 `streamerMode` 예시는 이 commit의 compile-time macro 구조와 다르므로 버전을 섞지 않는다.

- [stream_config.h](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/wifi-img-streamer/stream_config.h)의 default는 QVGA/324×244, START_STOP, encoding0/RAW, sensor frame-rate override0이다. 이는 [제조사 mono sensor의 유효320×320](https://www.bitcraze.io/documentation/hardware/ai_deck_mono_camera/ai_deck_mono_camera-datasheet.pdf) 및 사용자 native320×320 입력 조건과 다른 소프트웨어 설정이다. 시뮬레이션을324×244로 바꾸지 않는다. 숫자 macro만320으로 바꿔 실제 sensor raster/DMA/출력·화각이 맞는다고 주장하지 않는다.
- [camera_pipeline.c](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/wifi-img-streamer/camera_pipeline.c)는 sensor configuration/AEG 초기화·capture/DMA buffer·대기/queue/drop을 다룬다. START_STOP와 PIPELINED(buffer3) 설정은 capture/처리 순서가 다르다. 자동 노출/queue/drop/rolling shutter를 현재 고정 지연50ms/global frame 렌더와 같다고 볼 수 없다.
- [wifi-img-streamer.c](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/wifi-img-streamer/wifi-img-streamer.c)는 encoding1에서 JPEG, 그 외 RAW를 보낸다. native mono320×320/20FPS의 RAW payload는320²×20=2.048MB/s=16.384Mbit/s D이며, CPX/Wi-Fi overhead/실제 안정 throughput U이다.20FPS를 검증한 하드웨어 사양으로 표기하지 않는다.
- [cpx_stream.c](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/wifi-img-streamer/cpx_stream.c)의 packed image header는 magic/width/height/depth/type/size이고 capture timestamp/sequence 필드가 없다. PC 수신시간을 capture time으로 간주하는 지연 보상은 근거가 부족하다. timestamp 전송/동기화의 추가 소프트웨어와 측정이 필요하며, 현재 simulator timestamp는S로 남긴다.

공식 예제를 읽기만 했으며 upstream 변경·실기 flash·실제 카메라/통신 검증은 하지 않았다.
