# reachy-mini

## 소개

reachy-mini는 소형 로봇 플랫폼 또는 소프트웨어 프로젝트로, 간단한 제어, 시뮬레이션, 또는 로봇 동작 테스트를 위해 설계되었습니다. 이 프로젝트는 사용자가 손쉽게 로봇의 동작을 제어하고, 다양한 실험을 수행할 수 있도록 직관적인 인터페이스와 확장 가능한 구조를 제공합니다.

## 주요 특징

- 간단한 설치 및 실행
- 모듈화된 구조로 손쉬운 확장성
- 다양한 로봇 동작 예제 제공
- 시뮬레이션 및 실제 하드웨어 연동 지원(필요시)

## 서브모듈

이 프로젝트는 Git Submodule을 사용하여 외부 저장소를 포함하고 있습니다.

### reachy_mini

- **경로**: `reachy_mini/`
- **브랜치**: `develop`
- **저장소**: https://github.com/orocapangyo/reachy_mini.git
- **목적**: Reachy Mini 로봇의 공식 Python SDK 및 시뮬레이션 프레임워크 (개발 버전)

### reachy_mini_stl_convexify

- **경로**: `reachy_mini_stl_convexify/`
- **브랜치**: `main`
- **저장소**: https://github.com/orocapangyo/reachy_mini_stl_convexify.git
- **목적**: Reachy Mini 로봇의 STL 파일을 convex hull로 변환하여 물리 시뮬레이션 성능 최적화

### 서브모듈 사용 방법

```bash
# 처음 클론할 때
git clone --recurse-submodules https://github.com/orocapangyo/reachy-mini.git

# 이미 클론한 경우 서브모듈 초기화
git submodule update --init --recursive

# 서브모듈 업데이트
git submodule update --remote
```

## 관련 영상

[![reachy-mini 소개 영상](https://img.youtube.com/vi/JvdBJZ-qR18/0.jpg)](https://youtu.be/JvdBJZ-qR18?si=qhe4JHv3QVOF-5la)

해당 영상을 클릭하면 YouTube에서 자세한 내용을 확인할 수 있습니다.

## 기여자 (Contributors)

| No. | 이름 | 이메일 | GitHub |
|-----|------|--------|---------|
| 1 | 하범수 | habemsu7@gmail.com | BeomsuHa |
| 2 | 최정호 | ho8909585y@gmail.com   |  |
| 3 | 이하빈 | ihb0126@gmail.com | yabeeu0126 |
| 4 | 나승원 | jws10375@gmail.com | lala4768 |
| 5 | 이세현 | a93450311@gmail.com | - |
| 6 | 임태양 | jennetime98@gmail.com | jenlime98 |
| 7 | 이주선 | km01049@gmail.com | km01049 |
| 8 | 서다원 | tjekdnjs96@gmail.com | seodawon |
| 9 | 정재욱 | m6488kk@gmail.com | - |
| 10 | 문진환 | jhmoon0224@gmail.com  | - |
