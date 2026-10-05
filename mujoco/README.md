# MuJoCo 모델 — RecurDyn `HumanoidUpperBody` 이식

`HumanoidUpperBody.rmd` 한 파일에서 URDF를 만들고, 그 URDF에서 MuJoCo MJCF를 만듭니다.
수치를 손으로 옮겨 적은 곳은 없습니다. 파서는 `../chrono/rmd_model.py`(MATLAB `parse_rmd.m`의 Python 이식)를 재사용합니다.

```
HumanoidUpperBody.rmd
   │  rmd_to_urdf.py
   ▼
urdf/humanoid_upper.urdf          트리 (링크 21 + 툴 프레임 2, SI)
urdf/humanoid_upper.loops.json    URDF에 못 담는 것: 루프 닫힘, 액추에이터, IC, 중력, 툴
meshes/*.stl                      CAD 메쉬, 링크 로컬 좌표 [m]
   │  urdf_to_mjcf.py
   ▼
mjcf/humanoid_upper.xml           상위 모델: base + 두 서브시스템 attach + 키프레임 (토크 모터)
mjcf/arm_RA.xml                   RightArm 서브시스템
mjcf/arm_LA.xml                   LeftArm 서브시스템
mjcf/scene.xml                    상위 모델 + 바닥/조명 (보기용)
mjcf/*_servo.xml                  슬라이더용 변형 (같은 구조): 위치 서보 + 중력보상
```

## 서브시스템 구성

MJCF는 rmd의 서브시스템(HumanoidUpperBody / RightArm / LeftArm)과 같은 모양으로 나뉩니다.

```xml
<!-- humanoid_upper.xml -->
<asset>
  <model name="RA" file="arm_RA.xml"/>
  <model name="LA" file="arm_LA.xml"/>
</asset>
<body name="base">
  <frame name="RA_mount" pos=".." quat="..">          <!-- RightArm 서브시스템 프레임 (T_B0) -->
    <attach model="RA" body="body0" prefix="RA_"/>
  </frame>
  <frame name="LA_mount" ...> <attach model="LA" body="body0" prefix="LA_"/> </frame>
</body>
```

- **팔 파일**은 서브시스템 자기 프레임에서 기술합니다(`body0`가 원점).
  이름에 좌우 접두사가 없고(`body3`, `joint3`, `motor3`, `tool`, `joint6_3`), 단독으로 열립니다(팔 하나 7.09 kg).
  자기 루프 구속, 모터, 센서를 함께 갖고 있습니다.
- **상위 파일**이 `<attach prefix="RA_">`로 접두사를 붙입니다. 그래서 조립된 모델의 이름은 전과 같고(`RA_joint3` 등),
  `verify.py`와 `view.py`는 고칠 것이 없습니다.
- 두 팔은 **별개 파일**입니다. 같은 파일을 두 번 붙이는 방식이 아닙니다.
  거울상인 데다 rmd 수치도 조금씩 다르기 때문입니다(body3 CoM, 손목 링크 치수).
- 키프레임은 상위 파일에만 있습니다. 팔 파일에 두면 attach할 때 `RA_rmd_ic` 같은 팔별 키가 따로 생깁니다.
- `compiler`/`option`/`default`는 모든 파일에 똑같이 들어 있습니다.
  그래서 attach할 때 충돌 경고가 없고, 팔 파일만 열어도 같은 조건으로 돕니다.

생성할 때마다, 조립된 분할 모델을 같은 URDF로 만든 **단일 파일 모델**(메모리에서만 컴파일)과 이름 기준으로 대조합니다.
대조 항목은 바디, 관절, 지오메트리, 사이트, 구속, 액추에이터, 센서, 키프레임, 옵션, 그리고 IC 자세의 운동학입니다.
결과는 **차이 0**입니다.

scene 파일은 `<include>`를 쓰지 않고, 상위 파일 내용에 바닥·조명을 합친 독립 파일로 생성합니다.
MuJoCo 3.13에서는 include된 파일 안의 `<model file>`을 상대경로(`mjcf/scene.xml`)로 열면
디렉터리가 두 번 붙어(`mjcf/mjcf/arm_RA.xml`) 열리지 않기 때문입니다.

```powershell
cd claude_ws\mujoco
python rmd_to_urdf.py      # rmd가 바뀌면 여기부터
python urdf_to_mjcf.py
python verify.py           # 19개 검사, 종료코드 0 = 통과
python view.py             # 뷰어 + 관절 슬라이더
```

## 슬라이더로 관절 움직이기

`python view.py`를 실행하면 창 오른쪽 패널의 **Control**에 관절마다 슬라이더가 하나씩 있습니다
(`RA_servo1..7`, `LA_servo1..7`). 값은 목표 관절각 [rad]이고, 실제 각도는 **Joint** 패널에 나옵니다.
**Simulation → Load key**로 `rmd_ic`(시작 자세)나 `assembly`(q = 0)를 부르면 자세와 슬라이더가 함께 초기화됩니다.
`--key assembly`로 q = 0에서 시작할 수도 있습니다. 바디를 더블클릭한 뒤 Ctrl + 우클릭 드래그로 밀어볼 수 있습니다.

이 모드는 `humanoid_upper_servo.xml`을 씁니다. 원본과 다른 점은 세 가지입니다.

- 모든 바디에 `gravcomp="1"`을 걸었습니다. MuJoCo가 각 CoM에 −m·g를 가하므로, 평행링크 바디까지 포함해 폐루프를 통해 중력이 정확히 상쇄됩니다.
- 모터 대신 `<position>` 서보를 씁니다. 게인은 `kp = M_ii·ω²`, `kv = 2√(kp·M_ii)`(임계감쇠)이고, 기본 4 Hz입니다(`urdf_to_mjcf.py --servo-hz`).
  `M_ii`는 두 키프레임에서 본 관절공간 관성의 최댓값입니다. 그래서 어깨는 kp ≈ 450, 손목은 ≈ 0.2 N·m/rad로, 모든 관절의 대역폭이 같습니다.
- **q6 슬라이더 범위가 제한됩니다.** 손목 평행사변형은 q6가 약 ±90°일 때 접혀서 특이자세가 되고,
  그 너머에서는 루프가 교차된 가지로 넘어갈 수 있습니다.
  생성기가 루프 야코비안의 최소 특이값으로 특이 각도를 찾아(RA −89.3°/+89.8°, LA −88.0°/+86.3°) 그보다 10° 안쪽으로 범위를 둡니다.
  나머지 관절은 rmd에 한계가 없어 ±π입니다.

`view.py --torque`는 검증 대상인 토크 모델을 엽니다. 이때 슬라이더 값은 토크 [N·m]이고,
`--torque --hold`를 주면 매 스텝 폐루프 중력보상을 계산해 넣습니다.

필요한 패키지는 `mujoco`(3.13에서 확인), `numpy`뿐입니다. 미리보기 그림을 만들 때만 `Pillow`가 필요합니다.

## 모델 규약

| 항목 | 내용 |
|---|---|
| q = 0 | rmd **조립자세**. 모든 조인트에서 I/J 마커가 위치·자세까지 일치하는 것을 생성기가 확인합니다 |
| 링크 프레임 | 직렬 체인 바디는 `Ai` 마커를 씁니다. 따라서 MuJoCo 바디 포즈가 MATLAB/C++의 `A_i`와 같습니다. 평행링크 바디는 들어오는 조인트의 I 마커, `base`는 파트 프레임(= 모델 전역)을 씁니다 |
| 관절축 | 전부 링크 +z입니다. RecurDyn 회전각(I 마커가 J에 대해 z로 도는 각)과 부호, 영점이 같습니다 |
| 트리 방향 | RecurDyn 규약대로 J = 부모, I = 자식입니다. 자식에 이미 부모가 있는 조인트는 루프 닫힘으로 분류합니다 |
| Ground | `base`가 Ground에 항등 변환으로 용접되어 있어서(확인함) 하나로 합쳤습니다. RA `body0`(원래 Ground에 고정)도 `base`에 붙였고, 동역학적으로 동일합니다 |
| 이름 | 링크는 `RA_body3`, 관절은 `RA_joint3`, 모터는 `RA_motor3`, 툴은 `RA_tool` 식입니다. URDF/MJCF 주석에 RecurDyn 원래 이름이 있습니다 |
| 초기자세 | 키프레임 `rmd_ic`는 조인트 `IC` 변위입니다(RA q4 = +90°, LA q4 = −90°). `assembly`는 q = 0입니다 |
| 액추에이터 | `RotationalAxial1..7` → `<motor gear=1>`, 단위 N·m. rmd의 FUNCTION은 현재 전부 `(0)`입니다 |
| 툴 | `body7.Cij` 마커입니다(= `params/arm_*.json`의 `tool`). site `RA_tool`/`LA_tool`에 framepos/framequat 센서를 달았습니다 |
| 접촉 | 끔. rmd에 접촉 정의가 없고, 이웃 링크 CAD가 조인트에서 서로 겹칩니다 |
| 마찰·감쇠·armature | 0. rmd 조인트에 정의가 없습니다 |
| 적분 | `implicitfast`, dt = 0.5 ms |

### 손목 평행링크 — 강체 합산하지 않고 폐루프로 풉니다

`body5 – body6 – body6_1 – body6_2`는 네 축이 모두 평행한 **평면 평행사변형**입니다.
URDF에는 트리(`body6 → body6_1 → body6_2`)만 들어갑니다. 루프를 닫는 `RevJoint6_3@RightArm` / `RevJoint8@LeftArm`은
MJCF의 `<equality connect>`로 넣었습니다. 평면 루프라 점 구속으로 충분하고, 면외 방향 행은 중복입니다.

MATLAB/C++은 이 두 바디를 호스트에 강체 합산했지만, 여기서는 RecurDyn처럼 실제 폐루프로 풉니다.

MuJoCo의 구속은 소프트 구속입니다(`solref 0.002 1`, `solimp 0.99 0.999`).
정지 유지 중 루프 오차는 0.02 µm이고, 토크 없이 떨어뜨리는 격렬한 운동에서는 최대 약 40 µm(링크 80 mm 기준)입니다.
dt 0.5 ms에서 이보다 딱딱하게 잡으면 발산합니다(`solimp 0.9999`에서 에너지 폭주를 확인했습니다).
더 조여야 하면 dt를 0.25 ms로 줄이고 `--loop-solref 0.0005 1`을 주십시오.

### rmd 데이터 이상 — `body6_1` 관성 (수정함)

`body6_1`(양팔 모두, 7.3 g)의 rmd 관성텐서는 주관성모멘트가 삼각부등식(A + B ≥ C)을 **6.3% 위반**합니다.
실제 강체로는 불가능한 값이고, 곱관성 부호 규약과 무관합니다(양쪽 부호 다 위반). CAD 질량특성 계산에서 생긴 것으로 보입니다.
RecurDyn은 이 값을 그대로 적분하지만 MuJoCo는 거부합니다.

생성기는 주축 프레임에서 **가장 작은 주관성모멘트 하나만** C − B까지 올립니다
(1.31e-7 → 5.15e-7 kg·m², 주축과 나머지 두 값은 그대로).
실행할 때 경고를 출력하고, 원본과 수정값은 `loops.json`의 `inertial.*.repaired`에 남깁니다.
CAD 쪽에서 원본을 고치는 것이 맞습니다.

### 수치 정밀도

MJCF는 `MjSpec.to_xml()`이 아니라 URDF를 직접 파싱해 17자리로 씁니다.
`to_xml()`은 유효숫자 6자리라 모든 프레임에 약 1e-6의 오차가 들어가기 때문입니다.

관성도 numpy로 고유분해해서 `diaginertia` + `quat`로 씁니다.
MuJoCo의 `fullinertia` 경로(`mju_eig3`)는 반복 근사라 텐서에 상대 약 1e-7의 오차를 남깁니다(body3에서 1.75e-9 kg·m²).

`urdf_to_mjcf.py`는 만든 MJCF를 **MuJoCo 자체 URDF 임포터**로 읽은 모델과 바디별로 대조한 뒤에야 끝납니다.
프레임·질량·관절축은 2.2e-15로 일치하고, 관성은 상대 1.1e-7로 일치합니다(임포터 쪽의 eig3 오차).

## 검증 (`verify.py`)

**A. 트리 — MATLAB 기준값 `params/ref_{RA,LA}.csv` 60케이스**

메모리 안에서 MATLAB과 똑같이 평행링크를 합산한 변형 모델을 만들어 비교합니다.
합산에는 수리 전의 원본 관성을 씁니다.

| 항목 | RA | LA |
|---|---|---|
| 툴 위치 [m] | 3.0e-15 | 3.2e-15 |
| 툴 자세 | 1.0e-14 | 1.0e-14 |
| 중력토크 [N·m] | 7.7e-14 | 8.3e-14 |
| 역동역학 [N·m] | 9.9e-14 | 9.9e-14 |
| 질량행렬 [kg·m²] | 2.6e-15 | 3.3e-15 |

URDF의 프레임, 관절축 부호, 질량, CoM, 곱관성 부호가 전부 MATLAB/C++(그리고 이들이 이미 대조를 마친 RecurDyn)과 같습니다.

**B. 폐루프 모델 (배포하는 그대로)**

| 항목 | 결과 |
|---|---|
| 루프 닫힘 잔차 (수동각 풀이 후) | 1.2e-15 m |
| 중력보상 후 수동관절에 남는 토크 | 3.5e-18 N·m |
| rmd IC 자세에서 루프 − 합산 토크 차이, q6 | **0.242 N·mm** (다른 관절 0) |
| IC 자세에서 루프 중력토크를 상수로 유지, 0.5 s | 처짐 8e-6 mm |
| 토크 없이 놓음, 0.5 s | RA 151.0 mm / LA 93.1 mm 처짐 |

q6의 0.242 N·mm가 이 모델의 핵심 근거입니다. 상위 README에 따르면 RecurDyn에 합산 토크를 넣고 돌린 해석이 q6에만 **0.246 N·mm**의 잔차를 남겼고,
그 원인을 평행링크 합산으로 추정했습니다. MuJoCo 폐루프가 같은 자리에서 같은 크기의 차이를 독립적으로 재현하므로,
RecurDyn과 같은 물리를 풀고 있다고 볼 수 있습니다.

놓았을 때 처짐은 MATLAB 합산 시뮬레이션(RA 155 / LA 93 mm)과 거의 같습니다.
1초 이상으로 늘리면 손목이 감기는 구간이 들어와 결과가 구속 파라미터에 민감해집니다(혼돈적 구간). 비교는 0.5초에서 합니다.

자세별 루프 − 합산 차이는 최대 2.9 N·mm이고, 대부분 q6에 몰립니다. `verify.py` 출력에서 표로 볼 수 있습니다.
평행사변형에서 `body6_2`는 body5가 아니라 body6과 같이 회전하고, `body6_1`은 회전 없이 평행이동하므로, 합산 모델은 이 두 바디의 운동을 근사한 것입니다.

**C. 서보 변형 (슬라이더 모드)**

| 항목 | 결과 |
|---|---|
| IC 자세에서 2 s 정지 유지 (gravcomp) | 8.0e-11 rad |
| ±0.6 rad 스텝, 1.5 s 후 추종오차 | 4.4e-9 rad (루프 오차 19 µm) |
| q6를 슬라이더 양 끝으로 보냈을 때 평행링크가 원래 가지에 머무는지 | 2.4e-3 rad |

## 파일

| 파일 | 역할 |
|---|---|
| `rmd_to_urdf.py` | rmd → URDF + STL + `loops.json`. 조립자세 q=0, 링크 프레임이 관절축 위에 있는지, 액추에이터 배선을 확인합니다 |
| `urdf_to_mjcf.py` | URDF + `loops.json` → 서브시스템 분할 MJCF (+ 서보 변형, scene). 단일 파일 모델과 MuJoCo 자체 URDF 임포터 양쪽과 대조합니다 |
| `verify.py` | 아래 19개 검사 |
| `loops.py` | 평행링크 루프 유틸: 수동각 풀이(`close_loops`), 폐루프 중력토크, 특이자세 탐색 |
| `view.py` | 대화형 뷰어. 기본은 슬라이더 모드, `--torque [--hold]`는 토크 모델 |
| `figures/pose_preview.png` | 조립자세 / IC 자세 렌더 (메쉬가 올바른 링크에 붙었는지 육안 확인용) |

메쉬 출처는 `motion_generation/viewer/cad/mesh/*.npz`입니다(없으면 `InverseKinematics/cad/mesh`).
181k 삼각형으로 감축한 근사이므로 치수 측정에는 쓰지 마십시오.

## 남은 일

- **관절 한계**: rmd에 정의가 없어서 URDF는 `continuous`, MJCF는 무제한입니다. 값이 정해지면 `rmd_to_urdf.py`에 입력 경로를 추가합니다.
- **`body6_1` 관성**: 위 수리는 임시방편입니다. CAD/RecurDyn 원본을 고치는 것이 맞습니다.
- **접촉**: 필요해지면 CAD 메쉬 대신 단순화한 충돌 형상을 따로 만들어야 합니다. 현재 메쉬는 조인트에서 서로 겹칩니다.
