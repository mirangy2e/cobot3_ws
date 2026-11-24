# Isaac Sim 로봇 팔 실습: M0609 큐브 pick and place

두산 M0609 로봇 팔 + OnRobot RG2 그리퍼로 큐브를 집어 옮기는 프로그램을 Isaac Sim 에서 직접 만든다.
Python 만 알면 시작할 수 있도록, 실습(Lesson) 11개를 차례로 하며 프로그램을 완성한다. L11 연습 문제를 끝내면 벽을 피해 큐브를 옮기는 pick and place 를 스스로 만든 것이다.

## 진행 순서
```
L1  앱 시작과 종료
L2  World 와 프레임 시간
L3  장면 객체
L4  로봇 불러오기와 관절 제어
L5  메인 루프와 Play / Stop
L6  Task 와 Controller
L7  그리퍼와 pick and place
L8  IK 로 관절 각도 구하기
L9  타겟 따라가기 (종합 미션)
L10 trajectory 로 직선 이동
L11 RRT 로 장애물 피하기
```

## 준비
- Isaac Sim 5.1 이 `~/isaacsim` 에 설치되어 있어야 한다
- 모든 스크립트는 Isaac Sim 의 파이썬으로 실행한다: `isaac_python 파일.py`
- 로봇 파일은 `isaacpjt/M0609/robot/`, `descriptor/` 에 있다 (수정하지 않는다)

## 자산 관리 원칙
**자산은 한 곳에 두고, 장면과 코드는 참조만 한다 (복사하지 않는다).**

```
M0609/
├── robot/  props/        자산: 다운로드한 원본. 고치지 않는다
├── scenes/               장면: 자산을 참조 + 장면만의 것 (Action Graph, 조명, 배치)
└── basic/  ros/          코드: 자산을 참조 (add_reference_to_stage)
```

| 단계 | 방법 | 확인 |
|---|---|---|
| ① 다운로드 (자산마다 한 번) | Content 탭에서 자산 열기 -> File > **Collect and Save As** -> `props/<이름>/` | 폴더 안에 usd 와 Materials 가 함께 있다 |
| ② GUI 에서 활용 | 장면을 `scenes/` 에 먼저 Save -> File > Add Reference -> `props/<이름>/<이름>.usd` 선택 (prim 이름 = 파일 이름) | Property > References 경로가 `../` 로 시작하는 상대 경로다 |
| ③ 코드에서 활용 | `add_reference_to_stage(usd_path=f"{M0609_DIR}/props/<이름>/<이름>.usd", prim_path="/World/<이름>")` | basic 의 로봇 불러오기와 같은 방법 |

- 하지 않는다: 자산 파일을 열고 Save As 로 장면 만들기 (자산이 장면 안에 **복사**되어, 원본을 고쳐도 반영되지 않는다)
- 하지 않는다: Content 탭의 원격 자산을 프로젝트 장면에 바로 넣기 (인터넷과 서버 버전에 묶인다. 잠깐 실험할 때만)
- 큐브 같은 단순한 도형은 자산 없이 코드로 만든다 (`DynamicCuboid`, `FixedCuboid`). 자산으로 관리하는 것은 로봇, 메시가 있는 소품, 환경이다
- 이렇게 하면 로봇이나 자산을 고쳤을 때 모든 장면과 코드에 한 번에 반영되고, `M0609/` 폴더째 다른 PC 로 옮겨도 열린다
- GUI 실습: `gui/GUI_Scene_Asset_Reference.md` (자산 참조로 작업 씬 만들기)

## 실습 (`isaacpjt/M0609/basic/`)
한 폴더에 Lesson 마다 파일 4개가 있다. 예) L1
| 파일 | 내용 |
|---|---|
| `L1_app_readme.md` | 학습 목표, 핵심 개념, 실행 방법, 실습, 연습 문제 (여기서 시작한다) |
| `L1_app.py` | 예제 (실행하고 코드를 읽는다) |
| `L1_app_answer.py` | 연습 문제 정답 코드 |
| `L1_app_answer.md` | 정답 코드와 해설 |

L9 는 종합 미션이라 예제와 연습 문제 없이 `L9_follow_target_readme.md` (요구 사항), `L9_follow_target.py` (완성 코드), `L9_follow_target_code.md` (해설) 3개다.

```bash
cd ~/cobot3_ws/isaacpjt/M0609/basic
isaac_python L1_app.py
```

모든 Lesson 은 창 모드로 실행한다. 창을 닫으면 스크립트가 끝난다.
