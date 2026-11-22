# PMSM GaN FOC

GaN 모터 드라이버를 직접 설계해서 PMSM(영구자석 동기 전동기)을 FOC(벡터 제어)로 구동하는 개인 학습 프로젝트

검증된 시판 드라이버로 FOC 펌웨어를 먼저 완성한 뒤, 직접 만든 GaN 인버터 보드로 옮겨 Si 드라이버와 비교하는 순서로 진행

## 목표

- FOC 전체 흐름(Clarke/Park 변환, SVPWM, 전류·속도·위치 루프) 직접 구현 및 튜닝
- 100V급 GaN 하프브리지(TI LMG2100R044) 기반 3상 인버터 설계 및 PCBA 제작
- PWM 주파수(20/40/80/100kHz)에 따른 효율·발열·전류 리플·스위칭 파형 비교
- 엔코더를 기준값으로 삼은 센서리스 관측기(SMO) 검증

## 단계 구성

| 단계 | 모터 | 드라이버 | 목적 |
|---|---|---|---|
| **0단계** | 시뮬레이션 모델 (SPMSM) | Python 평균값 인버터 모델 | FOC 핵심 블록 직접 구현, Claude와 함께 학습 |
| **1단계** | iFlight GM4108H-120T (AS5048A 엔코더 포함) | ST X-NUCLEO-IHM07M1 | FOC 펌웨어 검증 (저전류, 안전) |
| **2단계** | 57BL 계열 48V 양축 홀센서 BLDC | ST X-NUCLEO-IHM08M1 | 실전 전류(5~10A), Si MOSFET 기준 데이터 확보 |
| **3단계** | 1·2단계 모터 | 자작 GaN 보드 (LMG2100R044 × 3) | GaN 드라이버 검증, Si vs GaN 비교 |

제어 MCU는 전 단계 공통으로 **STM32 NUCLEO-G474RE** 사용

## 진행 상황

- [x] 개념 정리 및 부품 선정
- [ ] 모터 이론 학습 ([체크리스트](study/checklist.md))
- [ ] 0단계: Python FOC 시뮬레이션 (TODO 1~7)
- [x] 1단계 부품 구매
- [ ] 마일스톤 1: 개루프(V/f) 모터 구동
- [ ] 마일스톤 2: AS5048A SPI 읽기, 전기각 정렬
- [ ] 마일스톤 3: 전류 오프셋 보정, 3상 전류 합 확인
- [ ] 마일스톤 4: 전류 루프 (Id = 0, Iq 지령)
- [ ] 마일스톤 5: 속도 루프, 위치 루프
- [ ] 마일스톤 6: GaN 보드 교체, 스위칭 파형 확인
- [ ] 마일스톤 7: 센서리스 관측기 검증

세부 계획은 [로드맵](docs/05-roadmap.md) 참고

## AI 활용

전 단계에서 Claude(AI)를 학습·디버깅 도우미로 활용, 도움받은 내용은 단계별 학습 기록에 그대로 기록

| 단계 | 기록 위치 |
|---|---|
| 이론 학습 | [study/README.md](study/README.md) (커리큘럼 초안, 공책 정리) |
| 0단계 | [stage0-simulation/README.md](stage0-simulation/README.md) (역할 구분표 + 학습 기록) |
| 1·2단계 | [firmware/README.md](firmware/README.md) 학습 기록 |
| 3단계 | [hardware/README.md](hardware/README.md) 학습 기록 |

- Claude가 작성한 코드는 파일 안에 `[제공됨]` 또는 주석으로 표시
- 설계 판단, 구현, 튜닝, 측정과 그 결과는 직접 수행

## 문서

| 문서 | 내용 |
|---|---|
| [01. PMSM과 FOC 기초](docs/01-pmsm-foc-basics.md) | PMSM 구조, FOC 흐름, Id=0·MTPA·약계자, 위치 검출 |
| [02. 최근 모터 제어 트렌드](docs/02-motor-control-trends.md) | SiC·GaN, 로봇 액추에이터, 원칩 통합, 고급 제어, AI |
| [03. 하드웨어 선정](docs/03-hardware-selection.md) | 모터·드라이버·MCU 선정 이유, 측정 장비 판단 |
| [04. 부품 리스트(BOM)](docs/04-bom.md) | 1단계 구매 품목, 장비, GaN 보드 계획 |
| [05. 로드맵과 안전 수칙](docs/05-roadmap.md) | 마일스톤, 학습 항목, 안전 수칙 |
| [모터 이론 학습 체크리스트](study/checklist.md) | 전동기 기초부터 GaN 드라이버까지 64개 학습 항목 |

## 레포 구성

```
.
├── docs/              설계 메모와 정리 문서
├── study/             모터 이론 학습 체크리스트와 공책 정리 노트
├── stage0-simulation/ Python FOC 시뮬레이션 실습 (0단계)
├── firmware/          STM32G474 펌웨어 (진행 예정)
└── hardware/          GaN 인버터 보드 KiCad 프로젝트 (진행 예정)
```
