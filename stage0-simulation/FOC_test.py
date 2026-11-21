"""
FOC (Field Oriented Control) 실습용 시뮬레이션 틀
=================================================

구성
----
[제공됨]  MotorParams      : 모터/인버터 파라미터
[제공됨]  PMSMPlant        : 3상 표면부착형 PMSM + 평균값 인버터 모델 ("실제 세계" 역할)
[제공됨]  run_simulation() : 제어 루프 타이밍, 로깅, 플롯
[제공됨]  test_*()         : 내가 구현한 함수가 맞는지 확인하는 테스트

[직접 구현]  ── 아래 TODO 를 순서대로 채우세요 ──
  TODO 1. clarke()          : abc  -> αβ
  TODO 2. park()            : αβ   -> dq
  TODO 3. inv_park()        : dq   -> αβ
  TODO 4. PIController      : PI 제어기 (+ 출력 제한, anti-windup)
  TODO 5. svpwm()           : αβ 전압 -> 3상 듀티 (처음엔 SPWM 으로 시작해도 OK)
  TODO 6. FOCController.current_loop() : 전류 제어 루프 조립
  TODO 7. FOCController.speed_loop()   : 속도 제어 루프

실습 단계 (STAGE 변수로 선택)
----------------------------
  STAGE = 1 : 좌표 변환 / SVPWM 단위 테스트만 실행
  STAGE = 2 : 전류 제어 (iq_ref 스텝 입력) -> id, iq 추종 확인
  STAGE = 3 : 속도 제어 (속도 스텝 + 부하 토크) -> 속도 추종 확인

규약 (테스트가 이 규약을 기준으로 검사합니다)
------------------------------------------
  * Clarke 변환은 "크기 불변(amplitude-invariant)" 형식 (계수 2/3)
  * 전기각 θe = 0 일 때 d축 = a상 축
  * a상 자속쇄교 λa = λ·cos(θe)  ->  역기전력 ea = -ωe·λ·sin(θe)
  * 토크 Te = 1.5 · p · λ · iq      (p: 극쌍수, 표면부착형이므로 릴럭턴스 토크 없음)
"""

import math
from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt


# =============================================================================
# 실습 설정
# =============================================================================
STAGE = 1

TWO_PI = 2.0 * math.pi
SQRT3 = math.sqrt(3.0)


# =============================================================================
# [제공됨] 모터 / 인버터 파라미터
# =============================================================================
@dataclass
class MotorParams:
    Rs: float = 0.5        # 상저항 [Ω]
    Ls: float = 1.0e-3     # 동기 인덕턴스 [H] (SPMSM: Ld = Lq = Ls)
    flux: float = 0.01     # 영구자석 자속쇄교 λ [Wb]
    p: int = 4             # 극쌍수 (pole pairs)
    J: float = 1.0e-5      # 관성 [kg·m²]
    B: float = 1.0e-5      # 점성 마찰 [N·m·s/rad]
    Vdc: float = 24.0      # DC 링크 전압 [V]
    I_max: float = 10.0    # 전류 제한 [A]


# =============================================================================
# [제공됨] 플랜트: 3상 PMSM + 평균값 인버터
#   - 입력 : 3상 듀티 (da, db, dc) ∈ [0, 1]
#   - 출력 : 3상 전류 (ia, ib, ic), 기계각 θm (엔코더), 기계 속도 ωm
#   - 내부적으로 상(phase) 좌표계에서 직접 적분 -> 좌표 변환 정답이 숨겨져 있음
# =============================================================================
class PMSMPlant:
    def __init__(self, prm: MotorParams):
        self.prm = prm
        self.ia = 0.0
        self.ib = 0.0
        self.wm = 0.0          # 기계 각속도 [rad/s]
        self.theta_m = 0.0     # 기계각 [rad]
        self.T_load = 0.0      # 부하 토크 [N·m]

    def _emf(self, theta_e, we):
        lam = self.prm.flux
        ea = -we * lam * math.sin(theta_e)
        eb = -we * lam * math.sin(theta_e - TWO_PI / 3)
        ec = -we * lam * math.sin(theta_e + TWO_PI / 3)
        return ea, eb, ec

    def step(self, duty, dt):
        prm = self.prm
        da, db, dc = (min(max(d, 0.0), 1.0) for d in duty)

        # 인버터 극전압 -> 중성점 기준 상전압 (Y결선, 중성점 비접지)
        va0, vb0, vc0 = da * prm.Vdc, db * prm.Vdc, dc * prm.Vdc
        vn = (va0 + vb0 + vc0) / 3.0
        va, vb = va0 - vn, vb0 - vn

        ic = -self.ia - self.ib
        theta_e = prm.p * self.theta_m
        we = prm.p * self.wm
        ea, eb, ec = self._emf(theta_e, we)

        # 전기 방정식: v = R i + L di/dt + e
        dia = (va - prm.Rs * self.ia - ea) / prm.Ls
        dib = (vb - prm.Rs * self.ib - eb) / prm.Ls

        # 토크 = 전기 출력 / 기계 속도  (ω=0 에서도 성립하도록 정리된 형태)
        Te = -prm.p * prm.flux * (
            self.ia * math.sin(theta_e)
            + self.ib * math.sin(theta_e - TWO_PI / 3)
            + ic * math.sin(theta_e + TWO_PI / 3)
        )
        dwm = (Te - self.T_load - prm.B * self.wm) / prm.J

        self.ia += dia * dt
        self.ib += dib * dt
        self.wm += dwm * dt
        self.theta_m = (self.theta_m + self.wm * dt) % TWO_PI
        return Te

    def measure(self):
        """센서 측정값: 3상 전류, 엔코더 기계각, 기계 속도"""
        return (self.ia, self.ib, -self.ia - self.ib), self.theta_m, self.wm


# =============================================================================
# TODO 1~3. 좌표 변환
# =============================================================================
def clarke(ia, ib, ic):
    """abc -> αβ (크기 불변 Clarke 변환)

    Returns: (i_alpha, i_beta)
    """
    # TODO 1: 구현하세요
    raise NotImplementedError("TODO 1: clarke()")


def park(alpha, beta, theta_e):
    """αβ -> dq (Park 변환). theta_e: 전기각 [rad]

    Returns: (d, q)
    """
    # TODO 2: 구현하세요
    raise NotImplementedError("TODO 2: park()")


def inv_park(d, q, theta_e):
    """dq -> αβ (역 Park 변환)

    Returns: (alpha, beta)
    """
    # TODO 3: 구현하세요
    raise NotImplementedError("TODO 3: inv_park()")


# =============================================================================
# TODO 4. PI 제어기
# =============================================================================
class PIController:
    def __init__(self, kp, ki, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, ref, meas, dt):
        """한 샘플 PI 연산.

        구현 포인트:
          - error = ref - meas
          - 적분항 누적 (dt 사용)
          - 출력 제한 (out_min ~ out_max)
          - anti-windup: 출력이 포화되면 적분이 계속 쌓이지 않게 처리
            (clamping 방식 또는 back-calculation 방식 중 선택)
        Returns: 제한된 출력
        """
        # TODO 4: 구현하세요
        raise NotImplementedError("TODO 4: PIController.update()")


# =============================================================================
# TODO 5. SVPWM
# =============================================================================
def svpwm(v_alpha, v_beta, vdc):
    """αβ 기준 전압 -> 3상 듀티 (da, db, dc), 각 값은 0~1.

    구현 아이디어 (쉬운 순서대로):
      (a) SPWM     : 역 Clarke 로 va, vb, vc 를 구한 뒤 d = 0.5 + v / Vdc
                     -> 선형 영역 최대 상전압 크기 Vdc/2
      (b) Min-Max 주입 SVPWM : (a)의 3상 전압에 영상분
                     v0 = -(max + min)/2 를 더한 뒤 듀티로 변환
                     -> 선형 영역 최대 상전압 크기 Vdc/√3 (약 15% 이득)
      (c) 섹터 판별 + T1/T0 계산하는 고전적인 SVPWM
    (b)와 (c)는 결과가 같습니다. 원하는 방식으로 구현해 보세요.
    """
    # TODO 5: 구현하세요
    raise NotImplementedError("TODO 5: svpwm()")


# =============================================================================
# TODO 6~7. FOC 제어기
# =============================================================================
class FOCController:
    def __init__(self, prm: MotorParams, f_current=10e3, f_speed=1e3):
        self.prm = prm
        self.Ts_i = 1.0 / f_current      # 전류 루프 주기
        self.Ts_w = 1.0 / f_speed        # 속도 루프 주기

        v_lim = prm.Vdc / SQRT3          # SVPWM 선형 영역 한계 (SPWM이면 Vdc/2)

        # ---------------------------------------------------------------
        # 게인 튜닝은 직접 해보세요. (초기값 0 -> 아무 동작 안 함)
        # 힌트: 전류 루프 대역폭 ωc [rad/s] 를 정하고
        #       PI 영점으로 플랜트 극(R/L)을 상쇄하는 방법을 찾아보세요.
        #       속도 루프 대역폭은 전류 루프보다 충분히(5~10배 이상) 낮게.
        # ---------------------------------------------------------------
        self.pi_id = PIController(kp=0.0, ki=0.0, out_min=-v_lim, out_max=v_lim)
        self.pi_iq = PIController(kp=0.0, ki=0.0, out_min=-v_lim, out_max=v_lim)
        self.pi_w = PIController(kp=0.0, ki=0.0, out_min=-prm.I_max, out_max=prm.I_max)

        # 기준값 / 모니터링용 내부 변수
        self.id_ref = 0.0
        self.iq_ref = 0.0
        self.id = self.iq = 0.0
        self.vd = self.vq = 0.0
        self.theta_e = 0.0

    def speed_loop(self, w_ref, wm_meas):
        """속도 제어 (Ts_w 주기로 호출됨). 결과로 self.iq_ref 를 갱신.

        - w_ref, wm_meas : 기계 각속도 [rad/s]
        - SPMSM 은 보통 id_ref = 0 (MTPA)
        """
        # TODO 7: 구현하세요
        raise NotImplementedError("TODO 7: FOCController.speed_loop()")

    def current_loop(self, i_abc, theta_m, wm):
        """전류 제어 (Ts_i 주기로 호출됨). 3상 듀티를 반환.

        순서:
          1) 엔코더 기계각 -> 전기각 θe          (self.theta_e 에 저장)
          2) i_abc --Clarke--> iαβ --Park--> id, iq   (self.id, self.iq 에 저장)
          3) PI 로 vd, vq 계산                   (self.vd, self.vq 에 저장)
             (선택) 전향 보상(decoupling): vd -= ωe·L·iq, vq += ωe·(L·id + λ)
                    -> 보상 후 다시 전압 제한 고려
          4) vd, vq --역 Park--> vαβ --SVPWM--> (da, db, dc)
        Returns: (da, db, dc)
        """
        # TODO 6: 구현하세요
        raise NotImplementedError("TODO 6: FOCController.current_loop()")


# =============================================================================
# [제공됨] 단위 테스트
# =============================================================================
def _close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_transforms():
    print("[TEST] 좌표 변환")
    # 1) 평형 3상 (크기 1, 각도 φ) -> αβ 크기 1, 각도 φ
    for phi in np.linspace(0, TWO_PI, 7):
        abc = (math.cos(phi), math.cos(phi - TWO_PI / 3), math.cos(phi + TWO_PI / 3))
        ab = clarke(*abc)
        assert _close(ab, (math.cos(phi), math.sin(phi))), f"clarke 오류 @ φ={phi:.2f}: {ab}"
    print("  clarke   OK")

    # 2) αβ 벡터가 θ 방향을 향하면 dq = (1, 0), θ+90° 방향이면 (0, 1)
    for th in np.linspace(0, TWO_PI, 7):
        dq = park(math.cos(th), math.sin(th), th)
        assert _close(dq, (1.0, 0.0)), f"park 오류 @ θ={th:.2f}: {dq}"
        dq = park(-math.sin(th), math.cos(th), th)
        assert _close(dq, (0.0, 1.0)), f"park 오류(q축) @ θ={th:.2f}: {dq}"
    print("  park     OK")

    # 3) 역변환 왕복
    for th in np.linspace(0, TWO_PI, 7):
        ab = inv_park(0.3, -0.7, th)
        assert _close(park(*ab, th), (0.3, -0.7)), f"inv_park 오류 @ θ={th:.2f}"
    print("  inv_park OK")


def test_pi():
    print("[TEST] PI 제어기")
    pi = PIController(kp=1.0, ki=10.0, out_min=-1.0, out_max=1.0)
    out = pi.update(0.5, 0.0, 0.01)
    assert abs(out - (0.5 + 10.0 * 0.5 * 0.01)) < 1e-3, f"PI 출력 이상: {out}"
    for _ in range(1000):                     # 오랫동안 포화
        out = pi.update(10.0, 0.0, 0.01)
    assert abs(out - 1.0) < 1e-9, "출력 제한이 동작하지 않음"
    out = pi.update(-0.1, 0.0, 0.01)          # 부호 반전 시 빠르게 빠져나와야 함
    assert out < 1.0, "anti-windup 미동작: 적분이 과도하게 쌓여 포화에서 못 빠져나옴"
    print("  PI       OK")


def test_svpwm():
    print("[TEST] SVPWM")
    vdc = 24.0
    for th in np.linspace(0, TWO_PI, 13):
        mag = 0.5 * vdc / SQRT3 * 1.5            # Vdc/2 보다 작은 크기 -> 어떤 방식이든 선형 영역
        va, vb = mag * math.cos(th), mag * math.sin(th)
        d = svpwm(va, vb, vdc)
        assert all(0.0 <= x <= 1.0 for x in d), f"듀티 범위 벗어남: {d}"
        # 선간전압으로 검증 (영상분은 상전압에 영향 없음)
        vab = (d[0] - d[1]) * vdc
        vab_ref = 1.5 * va - (SQRT3 / 2) * vb   # va - vb (역 Clarke 기준)
        assert abs(vab - vab_ref) < 1e-6, f"선간전압 불일치 @ θ={th:.2f}: {vab:.3f} vs {vab_ref:.3f}"
    print("  svpwm    OK")


# =============================================================================
# [제공됨] 시뮬레이션
# =============================================================================
def run_simulation(stage, t_end=0.3, dt_sim=5e-6):
    prm = MotorParams()
    plant = PMSMPlant(prm)
    ctrl = FOCController(prm)

    n_i = int(round(ctrl.Ts_i / dt_sim))   # 전류 루프 1회당 플랜트 스텝 수
    n_w = int(round(ctrl.Ts_w / ctrl.Ts_i))  # 속도 루프 1회당 전류 루프 횟수
    n_steps = int(t_end / ctrl.Ts_i)

    log = {k: np.zeros(n_steps) for k in
           ["t", "ia", "ib", "ic", "id", "iq", "id_ref", "iq_ref",
            "vd", "vq", "wm", "w_ref", "Te", "da", "db", "dc"]}

    w_ref = 0.0
    for k in range(n_steps):
        t = k * ctrl.Ts_i

        # ----- 시나리오 -----
        if stage == 2:
            ctrl.id_ref = 0.0
            ctrl.iq_ref = 0.0 if t < 0.01 else (1.0 if t < 0.15 else 2.0)
            plant.T_load = 8e-4 * plant.wm           # 팬 부하 (속도 비례) -> 폭주 방지
        elif stage == 3:
            w_ref = 200.0 if t >= 0.01 else 0.0      # [rad/s] ≈ 1910 rpm
            plant.T_load = 0.05 if t >= 0.15 else 0.0

        # ----- 제어기 -----
        i_abc, theta_m, wm = plant.measure()
        if stage == 3 and k % n_w == 0:
            ctrl.speed_loop(w_ref, wm)
        duty = ctrl.current_loop(i_abc, theta_m, wm)

        # ----- 플랜트 (제어 주기 동안 듀티 유지, ZOH) -----
        for _ in range(n_i):
            Te = plant.step(duty, dt_sim)

        # ----- 로깅 -----
        for key, val in zip(
            ["t", "ia", "ib", "ic", "id", "iq", "id_ref", "iq_ref",
             "vd", "vq", "wm", "w_ref", "Te", "da", "db", "dc"],
            [t, *i_abc, ctrl.id, ctrl.iq, ctrl.id_ref, ctrl.iq_ref,
             ctrl.vd, ctrl.vq, wm, w_ref, Te, *duty]):
            log[key][k] = val

    return log


def plot(log, stage):
    t = log["t"] * 1e3
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(f"FOC Simulation - STAGE {stage}")

    ax[0].plot(t, log["ia"], label="ia")
    ax[0].plot(t, log["ib"], label="ib")
    ax[0].plot(t, log["ic"], label="ic")
    ax[0].set_ylabel("Phase current [A]")

    ax[1].plot(t, log["id"], label="id")
    ax[1].plot(t, log["iq"], label="iq")
    ax[1].plot(t, log["id_ref"], "--", label="id_ref")
    ax[1].plot(t, log["iq_ref"], "--", label="iq_ref")
    ax[1].set_ylabel("dq current [A]")

    ax[2].plot(t, log["vd"], label="vd")
    ax[2].plot(t, log["vq"], label="vq")
    ax[2].set_ylabel("dq voltage [V]")

    ax[3].plot(t, log["wm"], label="ωm")
    if stage == 3:
        ax[3].plot(t, log["w_ref"], "--", label="ω_ref")
    ax[3].set_ylabel("Speed [rad/s]")
    ax[3].set_xlabel("Time [ms]")

    for a in ax:
        a.grid(True)
        a.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


# =============================================================================
# main
# =============================================================================
if __name__ == "__main__":
    try:
        test_transforms()
        test_pi()
        test_svpwm()
        print("\n모든 단위 테스트 통과!\n")
        if STAGE >= 2:
            print(f"STAGE {STAGE} 시뮬레이션 실행 중...")
            log = run_simulation(STAGE)
            plot(log, STAGE)
        else:
            print("STAGE = 2 로 바꾸고 전류 제어를 시작해 보세요.")
    except NotImplementedError as e:
        print(f"\n>>> 아직 구현되지 않음: {e}")
    except AssertionError as e:
        print(f"\n>>> 테스트 실패: {e}")
