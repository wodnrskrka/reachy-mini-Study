## 3주차: 고급 동작 제어

### 학습 목표

- 다양한 보간(interpolation) 방법 활용
- 실시간 동작 제어
- 모터 상태 관리

---

## 1. 다양한 보간(Interpolation) 방법

### 1.1 보간이란?

보간은 시작 위치에서 목표 위치까지 로봇 관절을 부드럽게 이동시키는 기법입니다. Reachy Mini는 여러 보간 방법을 지원합니다.

### 1.2 선형 보간 (Linear Interpolation)

가장 기본적인 보간 방법으로, 시작점과 끝점을 직선으로 연결합니다.

```python
from reachy_mini import ReachyMini

reachy = ReachyMini()

# 선형 보간으로 목 움직이기
reachy.neck.goto(
    goal_positions={'pitch': 20, 'yaw': 15},
    duration=2.0,
    interpolation_mode='linear'
)
```

**특징:**
- 단순하고 예측 가능
- 속도가 일정
- 급격한 시작/정지로 인한 떨림 가능성

### 1.3 최소 저크 보간 (Minimum Jerk)

부드러운 가속과 감속을 제공하여 자연스러운 움직임을 만듭니다.

```python
# 최소 저크 보간 (기본값)
reachy.neck.goto(
    goal_positions={'pitch': -10, 'yaw': -10},
    duration=1.5,
    interpolation_mode='minimum_jerk'  # 기본값
)
```

**특징:**
- 부드러운 시작과 종료
- 가속도 변화(jerk)를 최소화
- 인간과 유사한 자연스러운 움직임
- 로봇 관절에 부담 감소

### 1.4 보간 모드 비교

```python
import time

# 같은 동작을 다른 보간 방법으로 실행
positions = {'pitch': 30, 'yaw': 0}

# 선형 보간
print("선형 보간 시작")
reachy.neck.goto(goal_positions=positions, duration=2.0, interpolation_mode='linear')
time.sleep(2.5)

# 최소 저크 보간
print("최소 저크 보간 시작")
reachy.neck.goto(goal_positions={'pitch': 0, 'yaw': 0}, duration=2.0, interpolation_mode='minimum_jerk')
```

---

## 2. 실시간 동작 제어

### 2.1 논블로킹 동작

`wait=False` 옵션을 사용하여 로봇이 움직이는 동안 다른 작업을 수행할 수 있습니다.

```python
# 논블로킹 모드로 동작 시작
reachy.neck.goto(
    goal_positions={'pitch': 20, 'yaw': 30},
    duration=3.0,
    wait=False  # 즉시 리턴
)

# 동작 실행 중에 다른 작업 수행 가능
print("목이 움직이는 동안 다른 작업 수행 중...")
time.sleep(1.0)
print("여전히 움직이는 중...")
```

### 2.2 동작 상태 모니터링

```python
from reachy_mini.reachy_mini import ReachyMini

reachy = ReachyMini()

# 논블로킹으로 동작 시작
reachy.neck.goto(
    goal_positions={'pitch': 30},
    duration=2.0,
    wait=False
)

# 동작 완료까지 상태 확인
while reachy.neck.is_moving():
    # 현재 위치 확인
    current_pitch = reachy.neck.pitch.present_position
    print(f"현재 pitch 각도: {current_pitch:.2f}도")
    time.sleep(0.1)

print("동작 완료!")
```

### 2.3 동시 다중 관절 제어

```python
# 여러 관절을 동시에 제어
reachy.neck.goto(
    goal_positions={
        'pitch': 15,
        'roll': 10,
        'yaw': -20
    },
    duration=2.0
)
```

### 2.4 연속 동작 시퀀스

```python
# 동작 시퀀스 정의
sequence = [
    {'pitch': 20, 'yaw': 30},
    {'pitch': -10, 'yaw': -30},
    {'pitch': 0, 'yaw': 0},
]

# 각 동작을 순차적으로 실행
for positions in sequence:
    reachy.neck.goto(
        goal_positions=positions,
        duration=1.5,
        wait=True  # 각 동작이 완료될 때까지 대기
    )
    time.sleep(0.5)  # 동작 사이 짧은 대기
```

---

## 3. 모터 상태 관리

### 3.1 컴플라이언트 모드

컴플라이언트 모드는 모터의 토크를 해제하여 수동으로 움직일 수 있게 합니다.

```python
# 모든 모터를 컴플라이언트 모드로 설정
reachy.neck.turn_off()

print("이제 로봇의 목을 손으로 움직일 수 있습니다.")
time.sleep(5)

# 모터 다시 활성화
reachy.neck.turn_on()
```

### 3.2 개별 모터 제어

```python
# 특정 모터만 컴플라이언트 모드로
reachy.neck.pitch.compliant = True

print("pitch 모터만 수동으로 움직일 수 있습니다.")
time.sleep(3)

# 다시 활성화
reachy.neck.pitch.compliant = False
```

### 3.3 모터 상태 확인

```python
# 모터 정보 출력
print(f"Pitch - 컴플라이언트: {reachy.neck.pitch.compliant}")
print(f"Pitch - 현재 위치: {reachy.neck.pitch.present_position:.2f}도")
print(f"Pitch - 목표 위치: {reachy.neck.pitch.goal_position:.2f}도")
print(f"Pitch - 현재 속도: {reachy.neck.pitch.present_speed:.2f}도/s")
print(f"Pitch - 부하: {reachy.neck.pitch.present_load:.2f}%")
print(f"Pitch - 온도: {reachy.neck.pitch.temperature}°C")
```

### 3.4 안전한 모터 관리

```python
def safe_motor_control():
    """안전하게 모터를 제어하는 예제"""
    try:
        # 모터 활성화
        reachy.neck.turn_on()

        # 동작 수행
        reachy.neck.goto(
            goal_positions={'pitch': 20, 'yaw': 15},
            duration=2.0
        )

    except Exception as e:
        print(f"오류 발생: {e}")

    finally:
        # 항상 컴플라이언트 모드로 종료
        reachy.neck.turn_off()
        print("모터 안전하게 종료됨")

safe_motor_control()
```

### 3.5 모터 온도 모니터링

```python
import time

def monitor_temperature(duration=10):
    """모터 온도를 주기적으로 확인"""
    start_time = time.time()

    while time.time() - start_time < duration:
        temp = reachy.neck.pitch.temperature
        print(f"모터 온도: {temp}°C")

        # 온도가 너무 높으면 경고
        if temp > 60:
            print("⚠️ 경고: 모터 온도가 높습니다!")
            reachy.neck.turn_off()
            break

        time.sleep(1)

monitor_temperature(duration=10)
```

---

## 4. 종합 실습 예제

### 4.1 부드러운 헤드 트래킹

```python
import math
import time

def smooth_head_tracking():
    """부드러운 헤드 트래킹 데모"""
    reachy.neck.turn_on()

    try:
        # 원형 패턴으로 움직이기
        num_points = 20
        radius = 20  # 각도
        duration_per_point = 0.3

        for i in range(num_points):
            angle = 2 * math.pi * i / num_points

            pitch = radius * math.sin(angle)
            yaw = radius * math.cos(angle)

            reachy.neck.goto(
                goal_positions={'pitch': pitch, 'yaw': yaw},
                duration=duration_per_point,
                interpolation_mode='minimum_jerk',
                wait=True
            )

        # 원래 위치로 복귀
        reachy.neck.goto(
            goal_positions={'pitch': 0, 'yaw': 0},
            duration=1.0
        )

    finally:
        reachy.neck.turn_off()

smooth_head_tracking()
```

### 4.2 리액티브 모션 제어

```python
def reactive_motion():
    """실시간 반응형 동작 제어"""
    reachy.neck.turn_on()

    try:
        # 좌우로 천천히 움직이며 모니터링
        reachy.neck.goto(
            goal_positions={'yaw': 30},
            duration=3.0,
            wait=False
        )

        # 동작 중 상태 모니터링
        while reachy.neck.is_moving():
            current_yaw = reachy.neck.yaw.present_position
            load = reachy.neck.yaw.present_load

            print(f"Yaw: {current_yaw:.1f}도, 부하: {load:.1f}%")

            # 부하가 높으면 동작 중지
            if abs(load) > 50:
                print("높은 부하 감지! 동작 중지")
                reachy.neck.yaw.goal_position = reachy.neck.yaw.present_position
                break

            time.sleep(0.1)

        # 원위치
        time.sleep(0.5)
        reachy.neck.goto({'yaw': 0}, duration=1.0)

    finally:
        reachy.neck.turn_off()

reactive_motion()
```

---

## 5. 스튜어트 플랫폼 (Stewart Platform) 모터 구조

### 5.1 스튜어트 플랫폼이란?

스튜어트 플랫폼은 6개의 선형 액추에이터(또는 회전 모터)를 사용하여 6자유도(6-DOF) 움직임을 구현하는 병렬 로봇 메커니즘입니다. Reachy Mini의 목(neck) 부분은 스튜어트 플랫폼 구조를 사용하여 3축 회전(pitch, roll, yaw)과 3축 병진(x, y, z) 움직임을 구현합니다.

**주요 특징:**
- 6개의 Dynamixel XL330-M288-T 모터 사용
- 높은 강성과 정밀도
- 컴팩트한 크기에서 복잡한 3D 움직임 구현
- 각 모터가 협력하여 최종 자세 제어

### 5.2 하드웨어 사양

```
모터 사양:
- 모델: Dynamixel XL330-M288-T
- 개수: 6개
- 모터 암 길이: 0.04m (40mm)
- 연결 로드 길이: 0.085m (85mm)
- 헤드 Z 오프셋: 0.177m (177mm)
```

### 5.3 각 모터의 역할과 위치

#### stewart_1 (모터 1)
```python
위치: [0.0206, 0.0218, 0.0] m
브랜치 프레임: closing_1_2
솔루션 패턴: 0 (짝수)

# 위치적 특징
- 전방 우측 영역에 위치
- 주로 전방 pitch와 우측 roll 움직임에 기여
```

#### stewart_2 (모터 2)
```python
위치: [0.0085, 0.0288, 0.0] m
브랜치 프레임: closing_2_2
솔루션 패턴: 1 (홀수)

# 위치적 특징
- 전방 중앙 영역에 위치
- 주로 전방 pitch와 yaw 회전에 기여
```

#### stewart_3 (모터 3)
```python
위치: [-0.0292, 0.0070, 0.0] m
브랜치 프레임: closing_3_2
솔루션 패턴: 0 (짝수)

# 위치적 특징
- 좌측 영역에 위치
- 주로 좌측 roll과 yaw 회전에 기여
```

#### stewart_4 (모터 4)
```python
위치: [-0.0292, -0.0070, 0.0] m
브랜치 프레임: closing_4_2
솔루션 패턴: 1 (홀수)

# 위치적 특징
- 좌측 하단 영역에 위치
- 모터 3과 대칭적으로 좌측 움직임 지원
```

#### stewart_5 (모터 5)
```python
위치: [0.0085, -0.0288, 0.0] m
브랜치 프레임: closing_5_2
솔루션 패턴: 0 (짝수)

# 위치적 특징
- 후방 중앙 영역에 위치
- 주로 후방 pitch와 yaw 회전에 기여
```

#### stewart_6 (모터 6)
```python
위치: [0.0206, -0.0217, 0.0] m
브랜치 프레임: passive_7_link_y
솔루션 패턴: 1 (홀수)

# 위치적 특징
- 후방 우측 영역에 위치
- 주로 후방 pitch와 우측 roll 움직임에 기여
```

### 5.4 모터 배치 패턴

```
        stewart_2
       /         \
   stewart_3    stewart_1
      |            |
   stewart_4    stewart_6
       \         /
        stewart_5

대칭 구조:
- 모터 1, 6: 우측 전후방
- 모터 2, 5: 중앙 전후방
- 모터 3, 4: 좌측 전후방
```

### 5.5 역기구학 (Inverse Kinematics)

목표 자세(pitch, roll, yaw)가 주어지면, 각 모터의 각도를 계산합니다.

```python
from reachy_mini import ReachyMini

reachy = ReachyMini()

# 목표 자세 설정
target_pose = {
    'pitch': 15.0,  # 위아래 회전
    'roll': 10.0,   # 좌우 기울기
    'yaw': -5.0     # 좌우 회전
}

# 6개 모터의 각도가 자동으로 계산되어 실행됨
reachy.neck.goto(
    goal_positions=target_pose,
    duration=2.0
)
```

**내부 동작:**
1. 목표 자세를 4x4 변환 행렬로 변환
2. 각 모터의 브랜치 위치와 모터 위치 계산
3. 기하학적 제약을 고려하여 각 모터 각도 산출
4. 솔루션 패턴(0 또는 1)에 따라 적절한 해 선택

### 5.6 정기구학 (Forward Kinematics)

6개 모터의 각도로부터 최종 자세를 계산합니다.

```python
# 현재 모터 각도 확인
motor_positions = []
for i in range(1, 7):
    motor_name = f"stewart_{i}"
    # 실제 모터 객체에 접근하여 위치 확인
    # (실제 구현은 내부 API에 따라 다를 수 있음)

# 정기구학으로 현재 자세 계산
# SDK 내부적으로 자동 처리됨
current_pitch = reachy.neck.pitch.present_position
current_roll = reachy.neck.roll.present_position
current_yaw = reachy.neck.yaw.present_position

print(f"현재 자세 - Pitch: {current_pitch:.2f}°, Roll: {current_roll:.2f}°, Yaw: {current_yaw:.2f}°")
```

### 5.7 솔루션 패턴의 의미

각 모터는 `solution` 값이 0 또는 1로 설정되어 있습니다. 이는 역기구학 계산 시 여러 해가 존재할 때 어떤 해를 선택할지 결정합니다.

```python
솔루션 패턴:
- solution = 0: 짝수 패턴 (stewart_1, 3, 5)
- solution = 1: 홀수 패턴 (stewart_2, 4, 6)

특징:
- 교대로 배치되어 구조적 안정성 확보
- 특이점(singularity) 회피
- 일관된 움직임 보장
```

### 5.8 변환 행렬 (T_motor_world)

각 모터는 월드 좌표계 대비 고유한 4x4 변환 행렬을 가집니다.

```python
# stewart_1의 변환 행렬 예시
T_motor_world = [
    [0.866, -0.500, -0.000, -0.010],  # X축 방향
    [0.000,  0.000,  1.000, -0.077],  # Y축 방향
    [-0.500, -0.866,  0.000,  0.037],  # Z축 방향
    [0.000,  0.000,  0.000,  1.000]   # 동차 좌표
]

# 이 행렬은 다음을 포함:
# - 회전 성분 (3x3 좌상단 블록)
# - 위치 성분 (3x1 우상단 열)
# - 모터의 공간상 방향과 위치를 정의
```

### 5.9 실습: 개별 모터 상태 확인

```python
from reachy_mini import ReachyMini
import time

reachy = ReachyMini()

# 모든 스튜어트 플랫폼 모터 상태 확인
def check_stewart_motors():
    """6개 스튜어트 모터의 상태를 확인"""
    reachy.neck.turn_on()

    print("=== 스튜어트 플랫폼 모터 상태 ===\n")

    # 목을 특정 자세로 이동
    reachy.neck.goto(
        goal_positions={'pitch': 20, 'roll': 10, 'yaw': 15},
        duration=2.0
    )

    time.sleep(2.5)

    # 각 모터 정보는 내부 API를 통해 접근
    # (실제 구현은 SDK 버전에 따라 다를 수 있음)
    print(f"목표 자세:")
    print(f"  Pitch: {reachy.neck.pitch.goal_position:.2f}°")
    print(f"  Roll: {reachy.neck.roll.goal_position:.2f}°")
    print(f"  Yaw: {reachy.neck.yaw.goal_position:.2f}°")

    print(f"\n현재 자세:")
    print(f"  Pitch: {reachy.neck.pitch.present_position:.2f}°")
    print(f"  Roll: {reachy.neck.roll.present_position:.2f}°")
    print(f"  Yaw: {reachy.neck.yaw.present_position:.2f}°")

    reachy.neck.turn_off()

check_stewart_motors()
```

### 5.10 주의사항

1. **작업 공간 제한**
   - 스튜어트 플랫폼은 물리적 제약으로 인한 작업 공간 제한이 있습니다
   - 극단적인 각도 조합은 특이점을 유발할 수 있습니다

2. **동시성 제어**
   - 6개 모터가 동시에 협력하여 움직임
   - 개별 모터를 직접 제어하지 말고 neck 인터페이스 사용

3. **안전 범위**
```python
# 권장 각도 범위
safe_ranges = {
    'pitch': (-30, 30),  # 도 단위
    'roll': (-20, 20),
    'yaw': (-45, 45)
}

def safe_goto(pitch, roll, yaw):
    """안전 범위 내에서만 이동"""
    if not (-30 <= pitch <= 30):
        print("⚠️ Pitch 범위 초과")
        return
    if not (-20 <= roll <= 20):
        print("⚠️ Roll 범위 초과")
        return
    if not (-45 <= yaw <= 45):
        print("⚠️ Yaw 범위 초과")
        return

    reachy.neck.goto(
        goal_positions={'pitch': pitch, 'roll': roll, 'yaw': yaw},
        duration=2.0
    )
```

---

## 6. 실습 과제

### 과제 1: 보간 방법 비교
선형 보간과 최소 저크 보간을 사용하여 같은 동작을 수행하고, 차이점을 관찰하세요.

**추천 예제:**
- [examples/goto_interpolation_playground.py](../../examples/goto_interpolation_playground.py)
  - 다양한 보간 방법(linear, minjerk, ease, cartoon)을 자동으로 비교
  - 각 방법의 차이를 시각적으로 확인 가능
  - `InterpolationTechnique` 열거형으로 모든 보간 방법 테스트

**실습 팁:**
```python
from reachy_mini.utils.interpolation import InterpolationTechnique

# 사용 가능한 모든 보간 방법 확인
for method in InterpolationTechnique:
    print(f"보간 방법: {method}")
    # 각 방법으로 동일한 동작 수행하고 비교
```

### 과제 2: 안전한 동작 제어
온도와 부하를 모니터링하며 안전하게 동작하는 프로그램을 작성하세요.

**추천 예제:**
- [examples/minimal_demo.py](../../examples/minimal_demo.py)
  - 기본적인 안전한 동작 패턴 (with 문 사용)
  - 연속적인 실시간 제어 예제
  - KeyboardInterrupt 처리

- [examples/reachy_compliant_demo.py](../../examples/reachy_compliant_demo.py)
  - try-except-finally 패턴 활용
  - 안전한 종료 처리 (컴플라이언트 모드로 복귀)

**실습 팁:**
```python
import time
from reachy_mini import ReachyMini

def safe_motion_with_monitoring():
    """온도와 부하를 모니터링하는 안전한 동작 제어"""
    with ReachyMini(media_backend="no_media") as mini:
        try:
            mini.goto_target(create_head_pose(), duration=1.0)

            # 동작 중 모니터링
            start_time = time.time()
            while time.time() - start_time < 10.0:
                # 온도 확인 (실제 구현은 SDK에 따라 다를 수 있음)
                # temp = mini.get_temperature()
                # if temp > 60:
                #     print("⚠️ 온도 경고!")
                #     break

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("사용자가 중단했습니다.")
        finally:
            print("안전하게 종료합니다.")
```

### 과제 3: 복잡한 동작 시퀀스
여러 관절을 조합하여 자연스러운 "고개 끄덕이기" 동작을 만들어보세요.

**추천 예제:**
- [examples/sequence.py](../../examples/sequence.py)
  - 복잡한 다단계 동작 시퀀스
  - Yaw, Pitch, Roll 각각의 사인파 움직임
  - 병진 운동과 회전 운동 조합
  - 안테나와 머리 동시 제어

- [examples/recorded_moves_example.py](../../examples/recorded_moves_example.py)
  - 사전 녹화된 복잡한 동작 재생
  - emotions 라이브러리에 고개 끄덕임 등의 감정 표현 포함

**실습 팁:**
```python
import time
import numpy as np
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose

def nodding_motion():
    """고개 끄덕이기 동작"""
    with ReachyMini(media_backend="no_media") as mini:
        # 초기 위치
        mini.goto_target(create_head_pose(), duration=1.0)
        time.sleep(1.0)

        # 고개 끄덕이기 (3회 반복)
        for _ in range(3):
            # 아래로
            mini.goto_target(
                create_head_pose(pitch=20, degrees=True),
                duration=0.5
            )
            time.sleep(0.6)

            # 위로
            mini.goto_target(
                create_head_pose(pitch=-10, degrees=True),
                duration=0.5
            )
            time.sleep(0.6)

        # 원위치
        mini.goto_target(create_head_pose(), duration=1.0)
```

### 과제 4: 스튜어트 플랫폼 이해
스튜어트 플랫폼의 6개 모터가 어떻게 협력하여 3D 움직임을 만드는지 관찰하고, 각 축(pitch, roll, yaw)을 개별적으로 움직여보며 모터들의 동작 패턴을 분석하세요.

**추천 예제:**
- [examples/gui_demos/mini_head_position_gui.py](../../examples/gui_demos/mini_head_position_gui.py)
  - GUI 슬라이더로 pitch, roll, yaw를 실시간 제어
  - 각 축의 개별 영향 관찰 가능
  - X, Y, Z 위치도 조정하여 6-DOF 전체 테스트

- [examples/sequence.py](../../examples/sequence.py)
  - 각 축을 순차적으로 움직이는 패턴
  - 2초씩 yaw, pitch, roll을 개별적으로 테스트

**실습 팁:**
```python
import time
import numpy as np
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose

def analyze_stewart_platform():
    """스튜어트 플랫폼 분석: 각 축을 개별적으로 테스트"""
    with ReachyMini(media_backend="no_media") as mini:
        # 초기 위치
        mini.goto_target(create_head_pose(), duration=1.0)
        time.sleep(1.5)

        print("=== Pitch 축 테스트 ===")
        for angle in [0, 15, -15, 0]:
            print(f"Pitch: {angle}도")
            mini.goto_target(
                create_head_pose(pitch=angle, degrees=True),
                duration=1.0
            )
            time.sleep(1.5)

        print("\n=== Roll 축 테스트 ===")
        for angle in [0, 10, -10, 0]:
            print(f"Roll: {angle}도")
            mini.goto_target(
                create_head_pose(roll=angle, degrees=True),
                duration=1.0
            )
            time.sleep(1.5)

        print("\n=== Yaw 축 테스트 ===")
        for angle in [0, 30, -30, 0]:
            print(f"Yaw: {angle}도")
            mini.goto_target(
                create_head_pose(yaw=angle, degrees=True),
                duration=1.0
            )
            time.sleep(1.5)

        print("\n=== 복합 움직임 테스트 ===")
        mini.goto_target(
            create_head_pose(pitch=15, roll=10, yaw=20, degrees=True),
            duration=2.0
        )
        time.sleep(2.5)

        # 원위치
        mini.goto_target(create_head_pose(), duration=1.0)
        print("\n분석 완료!")

if __name__ == "__main__":
    analyze_stewart_platform()
```

**관찰 포인트:**
1. **Pitch 움직임**: 어떤 모터들이 주로 작동하는가?
2. **Roll 움직임**: 좌우 대칭 모터의 역할은?
3. **Yaw 움직임**: 모든 모터가 어떻게 협력하는가?
4. **복합 움직임**: 여러 축을 동시에 움직일 때 모터 간 조정은?

**추가 참고 예제:**
- [examples/look_at_image.py](../../examples/look_at_image.py): 실시간 목표 지점 추적
- [examples/debug/body_yaw_test.py](../../examples/debug/body_yaw_test.py): Body yaw와 head 움직임 조합

---

## 참고 자료

- [Reachy Mini Python SDK 문서](reachy_mini/docs/SDK/python-sdk.md)
- [보간 알고리즘 상세](reachy_mini/docs/SDK/core-concept.md)
- [모터 제어 가이드](reachy_mini/docs/platforms/reachy_mini/usage.md)
- [하드웨어 사양](reachy_mini/docs/platforms/reachy_mini/hardware.md)
- [운동학 데이터](reachy_mini/src/reachy_mini/assets/kinematics_data.json)

