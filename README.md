# phone_teleop

안드로이드 폰의 조그 버튼으로 휴머노이드 상체 양팔(RA/LA, 각 7축)의 끝단을 움직이는 텔레오퍼레이션입니다.
현재는 **MuJoCo 시뮬레이션 전용**입니다.

- 폰 앱: 가로 화면에서 왼쪽 절반은 LA, 오른쪽 절반은 RA 조그 패드입니다(X±·Y±·Z±·Rx±·Ry±·Rz±). 모션 센서는 쓰지 않습니다.
- 연결: **USB**(`adb reverse`)와 **Wi-Fi TCP**(UDP 비컨으로 PC 자동 탐색)를 모두 지원합니다.
- PC: 버튼 상태로 끝단 목표를 적분하고, DLS IK를 풀어 MuJoCo 위치 서보 모델을 구동합니다. 수신이 0.2초 끊기면 정지합니다.

## 구성

| 폴더 | 내용 |
|---|---|
| [`teleop/`](teleop/) | PC 프로그램(`teleop_sim.py`), 수신 서버, 테스트 클라이언트, 안드로이드 앱 소스와 빌드 스크립트. **사용법은 [teleop/README.md](teleop/README.md)** |
| [`mujoco/`](mujoco/) | RecurDyn 모델에서 생성한 MuJoCo 모델(MJCF, URDF, STL 메쉬)과 손목 평행링크 루프 유틸. 설명은 [mujoco/README.md](mujoco/README.md) |
| [`recurdyn/`](recurdyn/) | RecurDyn 원본: `HumanoidUpperBody.rmd`(기구학·질량의 단일 진실원, 텍스트), `.rdyn`(RecurDyn V9R5 모델), `.x_t`(Parasolid CAD) |
| [`docs/`](docs/) | 폰 텔레오퍼레이션 검토 문서(센서 기반 방식 검토부터 조그 방식 결정까지) |

## 빠른 시작

```powershell
pip install mujoco numpy
cd teleop
python android\build_apk.py --install   # Android SDK + Android Studio JBR 필요, Gradle 불필요
python teleop_sim.py                     # 뷰어가 열리고 USB / Wi-Fi 접속을 기다림
```

폰 앱에서 USB 또는 Wi-Fi를 고르고 **연결**을 누른 뒤, 버튼을 누르고 있는 동안 팔이 움직입니다.
폰 없이 시험하려면 `python fake_phone.py --arm both`를 실행하세요.

## 참고

- `mujoco/`의 생성 결과(`mjcf/`, `urdf/`, `meshes/`)는 그대로 쓸 수 있습니다.
  다시 생성(`rmd_to_urdf.py`)하거나 검증(`verify.py`)하는 스크립트는 원래 작업공간의 다른 폴더(`chrono/rmd_model.py`, `params/`, 메쉬 원본)를 참조하므로, 이 저장소만으로는 실행되지 않습니다.
- 확인한 환경: Windows 11, Python 3.14, MuJoCo 3.13. 폰은 Galaxy A15(Android 16)와 Galaxy S25에서 확인했습니다.
- 단위: RecurDyn 모델은 MMKS(mm, kg, N·mm)이고, MuJoCo 모델과 코드는 SI(m, kg, rad)입니다.
