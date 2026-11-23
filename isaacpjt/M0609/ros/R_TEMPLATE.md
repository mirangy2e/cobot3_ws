---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page:
---

# R?. <주제>: <한 줄로 무엇을 하는지>

## 📚 학습 목표
- <무엇을> <코드로/명령으로> <할 수 있다>
- <확인 방법> 으로 <동작을> 확인할 수 있다
- <이전 강의(GUI 또는 L 시리즈)와의 차이를> 설명할 수 있다

## 🔑 핵심 개념
1. **<개념 1>**
   - <한두 문장 설명>
   - <필요하면 표>

   | 항목 | 내용 |
   |---|---|
   | | |

2. **<개념 2: 예제 코드의 핵심 부분>**
   예제에서 <주제>를 다루는 부분은 아래가 전부다.

   ```python
   # 핵심 코드 10~20줄만 발췌 (전체 코드는 학습 내용 1)
   ```

   | 코드 | 역할 |
   |---|---|
   | `<함수/노드 이름>` | <역할> |

   > 💡 <헷갈리기 쉬운 점 한 가지만, 필요할 때만>

## ☑️ 사전 준비
1. **작업 씬 / 이전 강의**
   - <이 강의에서 쓰는 씬 또는 앞 강의 결과>

   ```
   ~/cobot3_ws/isaacpjt/M0609/scenes/<씬 이름>.usd
   ```

2. **파일 위치**

   ```
   ~/cobot3_ws/isaacpjt/M0609/ros/
   ├── R?_<이름>.py          예제
   └── R?_<이름>_answer.py   연습 문제 정답
   ```

3. **ROS2 환경변수 (Isaac Sim 터미널)**

   ```bash
   export ROS_DOMAIN_ID=50
   export ROS_DISTRO=jazzy
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
   isaac_ros
   ```

## 💡 학습 내용
1. **예제 실행**
   <예제가 무엇을 하는지 한 줄>

   ### R?_<이름>.py

   ```python
   # 전체 코드 (py 파일과 똑같이)
   ```

   ```bash
   cd ~/cobot3_ws/isaacpjt/M0609/ros
   isaac_python R?_<이름>.py
   ```

   - 확인 포인트: <Isaac Sim 창/터미널에서 보이는 것>

2. **ROS2 쪽에서 확인**
   새 터미널에서 <토픽/서비스/TF> 를 확인한다.

   ```bash
   ros_set
   ros2 topic list
   ros2 topic echo /<토픽> --once
   ```

   - 확인 포인트: <출력에서 볼 값>

   ```
   <출력 예>
   ```

3. **실습: <값 바꿔 보기>**
   <코드의 어느 값을 바꾸고, 무엇이 달라지는지 확인>

   ```python
   <바꿀 줄>
   ```

   - 확인 포인트: <달라진 결과>

## ✏️ 연습 문제

### 시나리오: <짧은 이름>

<무엇을 만들어야 하는지 1~2문장. 구현 방법은 쓰지 않는다>

- <요구 조건 1>
- <요구 조건 2>
- <확인 방법>

```
<출력 예>
```

## 📝 요약 정리

```
<단계 1> → <단계 2> → <단계 3>
    → <ROS 쪽 확인>
```

### 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `ros2 topic list` 에 토픽이 없음 | Play 상태가 아니거나 `ROS_DOMAIN_ID` 가 다름 | Play 상태, 두 터미널의 `echo $ROS_DOMAIN_ID` 확인 |
| | | |

## 📎 참고 문서
- <공식 튜토리얼 이름> — https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/<파일>.html
