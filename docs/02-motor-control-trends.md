# 02. 최근 모터 제어 트렌드 (2026년 기준)

제어 이론의 기본은 여전히 FOC, 최근 변화는 그 위에서 진행 중

- SiC·GaN으로 **더 빠르고 작게**
- 원칩 통합으로 **더 쉽게**
- AI와 관측기로 **더 똑똑하게, 센서 없이**

## 1. SiC·GaN 와이드밴드갭 반도체

가장 큰 변화는 전력 소자 쪽, 실리콘 대비 낮은 스위칭 손실, 높은 스위칭 주파수, 우수한 열 특성으로 인버터 구조 자체가 변화하는 중

- **SiC**: 800V 전기차, 산업용 서보 등 고전압 영역, 모터 일체형 서보 드라이브 실현의 핵심
- **GaN**: 48V 이하 저전압 영역, 출력 밀도, 부드러운 토크, 정밀 제어가 중요한 로봇·드론용 드라이브에 적합

## 2. 휴머노이드 로봇·드론용 소형 고밀도 액추에이터

최근 수요를 가장 크게 견인하는 분야

- FET, 게이트 드라이버, 보호 회로를 한 패키지에 넣은 GaN 파워 IC로 로봇 관절·손가락·선형 액추에이터 모듈 소형화 용이
- 주요 반도체 업체들의 휴머노이드·드론용 GaN 모터 드라이브 레퍼런스 디자인 출시 증가

## 3. 원칩 통합

MCU, 게이트 드라이버, 전력 소자, 센서리스 알고리즘을 하나의 IC로 통합하는 추세, 소형 BLDC 구동은 칩 하나로 해결되는 방향

## 4. 고급 제어 알고리즘과 센서리스

PI 기반 FOC를 넘어서는 방법에 대한 활발한 연구

- 모델 예측 제어(MPC), 비선형 제어
- 슬라이딩 모드 제어(SMC): PMSM의 비선형성, 결합 특성, 파라미터 민감성 대응에 강점
- 외란 관측기, 센서리스 상태 추정
- 멀티레벨 인버터 제어

## 5. AI 접목과 예지보전

- 최근 특허 동향의 주요 흐름: AI 보조 제어, 가변 자화, 멀티포트 모터 구조, 실시간 자석 상태 진단
- MCU에서 직접 실행하는 엣지 AI(TinyML 등) 기반 고장 감지 사례 증가
- 현재는 FOC 대체보다는 **게인 자동 튜닝, 파라미터 추정, 이상 진단** 등 보조 역할 위주

## 6. 희토류 리스크 대응

관세와 공급망 문제로 희토류 자석 비용 부담 증가, 이에 따라 다음 기술이 함께 주목받는 중

- 자석 사용량 저감 설계
- 자석 감자(demagnetization) 감시·진단
- 동기 릴럭턴스 모터(SynRM) 등 희토류 미사용 대안

## 참고 자료

- [MDPI Machines 특집호: Recent Advances in Control for PMSM Drives](https://www.mdpi.com/journal/machines/special_issues/C647B2ONOZ)
- [Sliding-Mode Control Strategies for PMSM speed control: A Comprehensive Review (arXiv)](https://arxiv.org/html/2510.18420v1)
- [PatSnap: Permanent magnet motor technology landscape 2026](https://www.patsnap.com/resources/blog/articles/permanent-magnet-motor-technology-landscape-2026/)
- [Infineon: The potential of SiC and GaN in motor drives](https://www.infineon.com/cms/en/product/promopages/wbg-for-motor-drives/)
- [Power Systems Design: The Power Electronics Enabling Motor Drives, Robotics, and Controls](https://www.powersystemsdesign.com/articles/the-power-electronics-enabling-motor-drives-robotics-and-controls/97/23656)
- [EEPower: Rethinking Power — GaN Innovation for Data Centers and Humanoid Robots](https://eepower.com/industry-articles/rethinking-power-gan-innovation-for-data-centers-and-humanoid-robots/)
- [TI TIDA-010936: 48V/16A small form factor three-phase GaN inverter reference design](https://www.ti.com/tool/TIDA-010936)
