// Tests de MissionStateMachine con un vehículo simulado muy simple que obedece las órdenes
// como lo haría PX4: armar, despegar (AUTO_TAKEOFF), ir a un punto (AUTO_LOITER), regresar (AUTO_RTL).

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "drone_core/geo.hpp"
#include "drone_mission/mission_state_machine.hpp"

using namespace drone_mission;
using drone_core::LatLon;
using drone_core::LocalProjection;

namespace
{

const LatLon kHome{43.6045, 1.4440};
constexpr double kHomeAlt = 150.0;

/** Vehículo cinemático: se mueve en línea recta hacia el objetivo a velocidad constante. */
class FakeVehicle
{
public:
  FakeVehicle()
  : proj_(kHome) {}

  bool armed{false};
  bool landed{true};
  bool failsafe{false};
  uint8_t nav_state{nav::kAutoLoiter};
  bool refuse_arm{false};
  bool home_known{true};         ///< false: home_position perdido hasta que PX4 lo fija al armar
  bool home_on_arming{true};
  bool ignore_goto{false};
  int drop_next_gotos{0};
  int gotos_received{0};
  double speed_mps{8.0};
  double e{0.0}, n{0.0}, h{0.0};  // posición local y altura relativa

  void apply(const Command & c)
  {
    switch (c.type) {
      case CommandType::ARM:
        if (!refuse_arm) {
          armed = true;
          if (home_on_arming) {
            home_known = true;
          }
        }
        break;
      case CommandType::TAKEOFF:
        nav_state = nav::kAutoTakeoff;
        tgt_e_ = e;
        tgt_n_ = n;
        tgt_h_ = c.alt_amsl_m - kHomeAlt;
        break;
      case CommandType::GOTO:
        ++gotos_received;
        if (ignore_goto || drop_next_gotos > 0) {
          if (drop_next_gotos > 0) {
            --drop_next_gotos;
          }
          break;  // orden perdida
        }
        nav_state = nav::kAutoLoiter;
        proj_.to_local(c.pos, tgt_e_, tgt_n_);
        tgt_h_ = c.alt_amsl_m - kHomeAlt;
        break;
      case CommandType::RTL:
        nav_state = nav::kAutoRtl;
        tgt_e_ = 0.0;
        tgt_n_ = 0.0;
        tgt_h_ = 0.0;
        break;
      case CommandType::NONE:
        break;
    }
  }

  void advance(double dt)
  {
    if (!armed) {
      return;
    }
    const double de = tgt_e_ - e, dn = tgt_n_ - n, dh = tgt_h_ - h;
    const double dist = std::sqrt(de * de + dn * dn + dh * dh);
    const double step = speed_mps * dt;
    if (dist <= step) {
      e = tgt_e_;
      n = tgt_n_;
      h = tgt_h_;
    } else {
      e += de / dist * step;
      n += dn / dist * step;
      h += dh / dist * step;
    }
    landed = (h < 0.1);
    if (nav_state == nav::kAutoTakeoff && std::fabs(h - tgt_h_) < 0.1) {
      nav_state = nav::kAutoLoiter;
    }
    if (nav_state == nav::kAutoRtl && landed && std::hypot(e, n) < 0.5) {
      armed = false;  // aterriza y desarma
    }
  }

  /** Pausa del piloto desde QGroundControl: PX4 se queda en el punto actual. */
  void pause_here()
  {
    tgt_e_ = e;
    tgt_n_ = n;
    tgt_h_ = h;
  }

  Inputs inputs(double now) const
  {
    Inputs in;
    in.now_s = now;
    in.status_valid = true;
    in.armed = armed;
    in.nav_state = nav_state;
    in.failsafe = failsafe;
    in.preflight_checks_pass = true;
    in.landed = landed;
    in.pos_valid = true;
    in.pos = proj_.to_global(e, n);
    in.alt_amsl_m = kHomeAlt + h;
    in.home_valid = home_known;
    in.home = kHome;
    in.home_alt_amsl_m = kHomeAlt;
    in.config_ok = true;
    in.setpoint_valid = armed;
    in.setpoint = proj_.to_global(tgt_e_, tgt_n_);
    in.setpoint_alt_amsl_m = kHomeAlt + tgt_h_;
    return in;
  }

private:
  LocalProjection proj_;
  double tgt_e_{0.0}, tgt_n_{0.0}, tgt_h_{0.0};
};

Plan demo_plan()
{
  const LocalProjection proj(kHome);
  Plan p;
  p.mission_id = "test";
  p.route = {proj.to_global(0.0, 200.0)};        // 200 m al norte
  p.drop_center = proj.to_global(150.0, 300.0);  // luego al noreste
  p.cruise_alt_m = 60.0;
  p.drop_alt_m = 20.0;
  return p;
}

/** Arnés: avanza máquina y vehículo en pasos de dt hasta que se cumple la condición o se agota el tiempo. */
struct Harness {
  MissionStateMachine sm{Params{}};
  FakeVehicle v;
  double t{0.0};
  static constexpr double kDt = 0.1;

  Harness()
  {
    std::string msg;
    EXPECT_TRUE(sm.set_plan(demo_plan(), msg)) << msg;
    tick();  // primera entrada para que el inicio tenga datos
  }

  void tick()
  {
    v.apply(sm.step(v.inputs(t)));
    v.advance(kDt);
    t += kDt;
  }

  template<typename Pred>
  bool run_until(Pred pred, double max_s)
  {
    const double end = t + max_s;
    while (t < end) {
      tick();
      if (pred()) {
        return true;
      }
    }
    return false;
  }

  bool run_until_state(State s, double max_s)
  {
    return run_until([&] {return sm.state() == s;}, max_s);
  }

  void start()
  {
    std::string msg;
    ASSERT_TRUE(sm.request_start(msg)) << msg;
  }
};

}  // namespace

TEST(MissionStateMachine, NominalMissionCompletes)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::TAKEOFF, 5.0));
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 30.0));
  EXPECT_EQ(hx.sm.waypoint_count(), 2U);
  ASSERT_TRUE(hx.run_until_state(State::APPROACH, 120.0));
  ASSERT_TRUE(hx.run_until_state(State::DROP, 60.0));
  EXPECT_NEAR(hx.v.h, 20.0, 2.0);  // a altura de suelta
  ASSERT_TRUE(hx.run_until_state(State::RETURN, 10.0));
  ASSERT_TRUE(hx.run_until_state(State::COMPLETED, 200.0));
  EXPECT_EQ(hx.sm.result(), Result::COMPLETED);
  EXPECT_FALSE(hx.v.armed);
}

TEST(MissionStateMachine, DescendsOnlyOverTheDropZone)
{
  // En crucero la altura se mantiene; solo se baja en APPROACH, ya sobre la zona.
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  double min_h_cruise = 1e9;
  ASSERT_TRUE(hx.run_until([&] {
      if (hx.sm.state() == State::CRUISE) {
        min_h_cruise = std::fmin(min_h_cruise, hx.v.h);
      }
      return hx.sm.state() == State::APPROACH;
    }, 120.0));
  EXPECT_GT(min_h_cruise, 57.0);
}

TEST(MissionStateMachine, StartIsRejectedWithBlockers)
{
  MissionStateMachine sm{Params{}};
  std::string msg;
  EXPECT_FALSE(sm.request_start(msg));  // sin datos
  Inputs in;
  in.status_valid = true;
  sm.step(in);
  EXPECT_FALSE(sm.request_start(msg));
  const auto b = sm.preflight_blockers(in);
  EXPECT_GE(b.size(), 4U);  // sin plan, sin config, prevuelo, posición, home
  Inputs none;
  EXPECT_EQ(sm.preflight_blockers(none).back(), "sin estado de PX4");
}

TEST(MissionStateMachine, AlreadyArmedOrFailsafeBlocksStart)
{
  Harness hx;
  Inputs in = hx.v.inputs(hx.t);
  in.armed = true;
  in.failsafe = true;
  in.home_valid = false;  // el home no bloquea el inicio
  const auto b = hx.sm.preflight_blockers(in);
  EXPECT_EQ(b.size(), 2U);
}

TEST(MissionStateMachine, ArmingTimeoutReturnsToPreflight)
{
  Harness hx;
  hx.v.refuse_arm = true;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::ARMING, 1.0));
  ASSERT_TRUE(hx.run_until_state(State::PREFLIGHT, 15.0));
  EXPECT_EQ(hx.sm.cause(), "PX4 no armó a tiempo");
}

TEST(MissionStateMachine, AbortInCruiseReturnsHome)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  std::string msg;
  ASSERT_TRUE(hx.sm.request_abort(msg));
  ASSERT_TRUE(hx.run_until_state(State::RETURN, 1.0));
  EXPECT_EQ(hx.v.nav_state, nav::kAutoRtl);
  ASSERT_TRUE(hx.run_until_state(State::COMPLETED, 200.0));
  EXPECT_EQ(hx.sm.result(), Result::ABORTED);
}

TEST(MissionStateMachine, AbortDuringArmingGoesBackToPreflight)
{
  Harness hx;
  hx.v.refuse_arm = true;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::ARMING, 1.0));
  std::string msg;
  ASSERT_TRUE(hx.sm.request_abort(msg));
  ASSERT_TRUE(hx.run_until_state(State::PREFLIGHT, 1.0));
}

TEST(MissionStateMachine, AbortRejectedWhenNothingToAbort)
{
  Harness hx;
  std::string msg;
  EXPECT_FALSE(hx.sm.request_abort(msg));
}

TEST(MissionStateMachine, PilotTakeoverLeadsToContingency)
{
  // SIM-19: el piloto cambia a modo Posición en crucero.
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.v.nav_state = nav::kPosCtl;
  ASSERT_TRUE(hx.run_until_state(State::CONTINGENCY, 10.0));
  EXPECT_NE(hx.sm.cause().find("toma de control"), std::string::npos);
  // En CONTINGENCY no se manda nada: el modo sigue siendo el del piloto.
  hx.run_until([] {return false;}, 5.0);
  EXPECT_EQ(hx.v.nav_state, nav::kPosCtl);
  hx.v.armed = false;  // el piloto aterriza y desarma
  ASSERT_TRUE(hx.run_until_state(State::COMPLETED, 1.0));
  EXPECT_EQ(hx.sm.result(), Result::CONTINGENCY);
}

TEST(MissionStateMachine, BriefModeGlitchIsTolerated)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.v.nav_state = nav::kPosCtl;
  hx.run_until([] {return false;}, 1.0);  // menos que mode_grace_s
  hx.v.nav_state = nav::kAutoLoiter;
  hx.run_until([] {return false;}, 5.0);
  EXPECT_EQ(hx.sm.state(), State::CRUISE);
}

TEST(MissionStateMachine, Px4FailsafeLeadsToContingency)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.v.failsafe = true;
  hx.v.apply(Command{CommandType::RTL, {}, 0.0});  // PX4 hace RTL por su cuenta
  ASSERT_TRUE(hx.run_until_state(State::CONTINGENCY, 1.0));
  ASSERT_TRUE(hx.run_until_state(State::COMPLETED, 200.0));
  EXPECT_EQ(hx.sm.result(), Result::CONTINGENCY);
}

TEST(MissionStateMachine, FailsafeDuringReturnDoesNotChangeState)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  std::string msg;
  hx.sm.request_abort(msg);
  ASSERT_TRUE(hx.run_until_state(State::RETURN, 1.0));
  hx.v.failsafe = true;
  hx.run_until([] {return false;}, 2.0);
  EXPECT_EQ(hx.sm.state(), State::RETURN);
}

TEST(MissionStateMachine, UnexpectedDisarmInFlight)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.v.armed = false;
  ASSERT_TRUE(hx.run_until_state(State::COMPLETED, 1.0));
  EXPECT_EQ(hx.sm.result(), Result::CONTINGENCY);
  EXPECT_EQ(hx.sm.cause(), "desarmado inesperado en vuelo");
}

TEST(MissionStateMachine, SlowLegTriggersReturn)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.v.speed_mps = 0.05;  // el vehículo casi no avanza
  ASSERT_TRUE(hx.run_until_state(State::RETURN, 200.0));
  EXPECT_EQ(hx.sm.cause(), "tramo de crucero demasiado lento");
}

TEST(MissionStateMachine, SlowTakeoffTriggersReturn)
{
  Harness hx;
  hx.v.speed_mps = 0.5;  // 60 m a 0,5 m/s = 120 s > 60 s
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::RETURN, 80.0));
  EXPECT_EQ(hx.sm.cause(), "despegue demasiado lento");
}

TEST(MissionStateMachine, SlowApproachTriggersReturn)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::APPROACH, 150.0));
  hx.v.speed_mps = 0.01;
  ASSERT_TRUE(hx.run_until_state(State::RETURN, 200.0));
  EXPECT_EQ(hx.sm.cause(), "aproximación demasiado lenta");
}

TEST(MissionStateMachine, CanRestartAfterCompletion)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::COMPLETED, 400.0));
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::ARMING, 1.0));
  EXPECT_EQ(hx.sm.result(), Result::NONE);
}

TEST(MissionStateMachine, PlanCannotChangeInFlight)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  std::string msg;
  EXPECT_FALSE(hx.sm.set_plan(demo_plan(), msg));
  EXPECT_FALSE(hx.sm.request_start(msg));
}

TEST(MissionStateMachine, IncompletePlanIsRejected)
{
  MissionStateMachine sm{Params{}};
  std::string msg;
  Plan p = demo_plan();
  p.drop_alt_m = 0.0;
  EXPECT_FALSE(sm.set_plan(p, msg));
  EXPECT_FALSE(sm.has_plan());
}

TEST(MissionStateMachine, CommandsAreNotSpammed)
{
  // En crucero la orden GOTO se repite como mucho cada command_resend_s.
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  int gotos = 0;
  for (int i = 0; i < 30; ++i) {  // 3 s
    const Command c = hx.sm.step(hx.v.inputs(hx.t));
    if (c.type == CommandType::GOTO) {
      ++gotos;
    }
    hx.t += Harness::kDt;
  }
  EXPECT_LE(gotos, 1);
}

TEST(MissionStateMachine, StateNamesAndCounters)
{
  EXPECT_STREQ(to_string(State::DROP), "DROP");
  EXPECT_STREQ(to_string(State::CONTINGENCY), "CONTINGENCY");
  EXPECT_STREQ(to_string(static_cast<State>(99)), "?");
  Harness hx;
  const uint32_t before = hx.sm.transition_count();
  hx.start();
  hx.run_until_state(State::TAKEOFF, 5.0);
  EXPECT_GT(hx.sm.transition_count(), before);
  EXPECT_EQ(hx.sm.previous_state(), State::ARMING);
}

TEST(MissionStateMachine, AllStateNames)
{
  const char * expected[] = {"PREFLIGHT", "ARMING", "TAKEOFF", "CRUISE", "APPROACH", "DROP", "RETURN",
    "CONTINGENCY", "COMPLETED"};
  for (int i = 0; i < 9; ++i) {
    EXPECT_STREQ(to_string(static_cast<State>(i)), expected[i]);
  }
}

TEST(MissionStateMachine, InvalidPositionNeverCountsAsArrival)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  // Con la posición no válida el vehículo puede estar encima del objetivo y no se da por llegado.
  for (int i = 0; i < 600; ++i) {
    Inputs in = hx.v.inputs(hx.t);
    in.pos_valid = false;
    hx.v.apply(hx.sm.step(in));
    hx.v.advance(Harness::kDt);
    hx.t += Harness::kDt;
  }
  EXPECT_EQ(hx.sm.state(), State::CRUISE);
  EXPECT_EQ(hx.sm.waypoint_index(), 0U);
}

TEST(MissionStateMachine, PilotPauseInCruiseLeadsToContingency)
{
  // AR-017 / SIM-19: el piloto pulsa Pausa en QGroundControl. El modo sigue siendo AUTO_LOITER,
  // pero la consigna de PX4 deja de ser el objetivo de la misión.
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.run_until([] {return false;}, 5.0);
  ASSERT_EQ(hx.sm.state(), State::CRUISE);
  hx.v.pause_here();
  ASSERT_TRUE(hx.run_until_state(State::CONTINGENCY, 3.0));
  EXPECT_NE(hx.sm.cause().find("consigna cambiada"), std::string::npos);
  // Y no se vuelve a mandar al dron hacia el objetivo.
  const int gotos = hx.v.gotos_received;
  hx.run_until([] {return false;}, 10.0);
  EXPECT_EQ(hx.v.gotos_received, gotos);
}

TEST(MissionStateMachine, LostGotoIsRetried)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::TAKEOFF, 5.0));
  hx.v.drop_next_gotos = 1;  // se pierde el primer GOTO del crucero
  ASSERT_TRUE(hx.run_until_state(State::APPROACH, 150.0));
  EXPECT_GE(hx.v.gotos_received, 3);  // 2 del crucero + el reintento
}

TEST(MissionStateMachine, GotoNeverAcceptedLeadsToContingency)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::TAKEOFF, 5.0));
  hx.v.ignore_goto = true;
  ASSERT_TRUE(hx.run_until_state(State::CONTINGENCY, 60.0));
  EXPECT_EQ(hx.sm.cause(), "PX4 no acepta la orden de reposicionamiento");
  EXPECT_EQ(hx.v.gotos_received, 3);
}

TEST(MissionStateMachine, InvalidSetpointAfterConfirmationLeadsToContingency)
{
  Harness hx;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::CRUISE, 40.0));
  hx.run_until([] {return false;}, 5.0);
  for (int i = 0; i < 30 && hx.sm.state() == State::CRUISE; ++i) {
    Inputs in = hx.v.inputs(hx.t);
    in.setpoint_valid = false;
    hx.v.apply(hx.sm.step(in));
    hx.v.advance(Harness::kDt);
    hx.t += Harness::kDt;
  }
  EXPECT_EQ(hx.sm.state(), State::CONTINGENCY);
}

TEST(MissionStateMachine, HomeLostBeforeStartIsRecoveredAtArming)
{
  // home_position se publicó antes de conectar el agente DDS: se recibe al armar.
  Harness hx;
  hx.v.home_known = false;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::TAKEOFF, 5.0));
  EXPECT_EQ(hx.sm.cause(), "armado y con home");
}

TEST(MissionStateMachine, ArmedWithoutHomeTimesOut)
{
  Harness hx;
  hx.v.home_known = false;
  hx.v.home_on_arming = false;
  hx.start();
  ASSERT_TRUE(hx.run_until_state(State::PREFLIGHT, 15.0));
  EXPECT_EQ(hx.sm.cause(), "armado pero sin home de PX4");
}
