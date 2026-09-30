// Tests de PayloadStateMachine (IR de la suelta: SR-PLD-002, SR-PLD-004; SIM-05, SIM-06, SIM-07).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "drone_payload/payload_state_machine.hpp"

using drone_payload::Command;
using drone_payload::CommandType;
using drone_payload::Inputs;
using drone_payload::Outcome;
using drone_payload::Params;
using drone_payload::PayloadStateMachine;
using drone_payload::Phase;

namespace
{

constexpr int32_t kHash = 12345;

drone_core::DropZone make_zone()
{
  drone_core::DropZone z;
  z.id = "z1";
  z.center = {43.6, 1.4};
  z.radius_m = 20.0;
  z.alt_min_m = 15.0;
  z.alt_max_m = 30.0;
  return z;
}

/** Entradas con todo en orden: sobre el centro de la zona, a 22 m sobre el hub, con carga. */
Inputs good(double now)
{
  Inputs in;
  in.now_s = now;
  in.armed = true;
  in.pos_valid = true;
  in.pos_age_s = 0.05;
  in.pos = {43.6, 1.4};
  in.home_alt_amsl_m = 150.0;
  in.alt_amsl_m = 172.0;
  in.eph_m = 1.0;
  in.epv_m = 1.0;
  in.home_valid = true;
  in.payload_valid = true;
  in.payload_present = true;
  in.dg_valid = true;
  in.dg_zone_hash = kHash;
  return in;
}

class PayloadTest : public ::testing::Test
{
protected:
  PayloadTest() : sm(Params{}) {}

  void start(double now = 0.0)
  {
    std::string msg;
    ASSERT_TRUE(sm.start(make_zone(), kHash, now, msg)) << msg;
    ASSERT_EQ(sm.phase(), Phase::WAITING_CONFIRMATION);
  }

  /** Confirma y avanza un paso con las entradas dadas. */
  Command confirm_and_step(const Inputs & in)
  {
    std::string msg;
    EXPECT_TRUE(sm.confirm(msg)) << msg;
    return sm.step(in);
  }

  PayloadStateMachine sm;
};

}  // namespace

TEST_F(PayloadTest, StartsIdleWithNoOutcome)
{
  EXPECT_EQ(sm.phase(), Phase::IDLE);
  EXPECT_EQ(sm.outcome(), Outcome::NONE);
  EXPECT_EQ(sm.last_outcome(), Outcome::NONE);
  EXPECT_FALSE(sm.release_commanded());
  EXPECT_EQ(sm.step(good(0.0)).type, CommandType::NONE);
}

TEST_F(PayloadTest, StartRejectsBusyAndInvalidZone)
{
  std::string msg;
  drone_core::DropZone bad = make_zone();
  bad.radius_m = 0.0;
  EXPECT_FALSE(sm.start(bad, kHash, 0.0, msg));
  bad = make_zone();
  bad.alt_min_m = bad.alt_max_m;
  EXPECT_FALSE(sm.start(bad, kHash, 0.0, msg));
  bad = make_zone();
  bad.center = {std::nan(""), 1.0};
  EXPECT_FALSE(sm.start(bad, kHash, 0.0, msg));
  EXPECT_EQ(sm.phase(), Phase::IDLE);

  start();
  EXPECT_FALSE(sm.start(make_zone(), kHash, 1.0, msg));
  EXPECT_EQ(msg, "ya hay una suelta en curso");
}

// SIM-05: sin confirmación no se suelta y se agota el tiempo.
TEST_F(PayloadTest, NoConfirmationTimesOutWithoutRelease)
{
  start(10.0);
  EXPECT_EQ(sm.step(good(10.0 + 119.9)).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::WAITING_CONFIRMATION);
  EXPECT_EQ(sm.step(good(10.0 + 120.0)).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::IDLE);
  EXPECT_EQ(sm.outcome(), Outcome::TIMEOUT);
  EXPECT_FALSE(sm.release_commanded());
}

TEST_F(PayloadTest, ConfirmOnlyWhileWaiting)
{
  std::string msg;
  EXPECT_FALSE(sm.confirm(msg));  // IDLE: una confirmación antigua no sirve para una suelta futura
  start();
  EXPECT_TRUE(sm.confirm(msg));
  EXPECT_FALSE(sm.confirm(msg));  // ya recibida
}

TEST_F(PayloadTest, StaleConfirmationBeforeStartDoesNotRelease)
{
  std::string msg;
  EXPECT_FALSE(sm.confirm(msg));
  start();
  EXPECT_EQ(sm.step(good(1.0)).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::WAITING_CONFIRMATION);
}

TEST_F(PayloadTest, NominalReleaseSendsOneCommandAndVerifies)
{
  start();
  EXPECT_EQ(confirm_and_step(good(5.0)).type, CommandType::RELEASE);
  EXPECT_EQ(sm.phase(), Phase::RELEASING);
  EXPECT_TRUE(sm.release_commanded());

  // No se reenvía la orden aunque pase el tiempo
  EXPECT_EQ(sm.step(good(5.1)).type, CommandType::NONE);
  sm.on_ack(true, 0);
  EXPECT_EQ(sm.step(good(5.2)).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::VERIFYING);

  Inputs released = good(5.5);
  released.payload_present = false;
  EXPECT_EQ(sm.step(released).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::IDLE);
  EXPECT_EQ(sm.outcome(), Outcome::RELEASED);
  EXPECT_EQ(sm.last_outcome(), Outcome::RELEASED);
  EXPECT_EQ(sm.message(), "carga liberada");
}

// SIM-07: la carga no se libera aunque PX4 acepte la orden.
TEST_F(PayloadTest, AcceptedButPayloadStaysIsNotReleased)
{
  start();
  confirm_and_step(good(5.0));
  sm.on_ack(true, 0);
  EXPECT_EQ(sm.step(good(5.2)).type, CommandType::NONE);
  sm.step(good(7.9));
  EXPECT_EQ(sm.phase(), Phase::VERIFYING);
  sm.step(good(8.0));  // 3 s tras la orden
  EXPECT_EQ(sm.outcome(), Outcome::NOT_RELEASED);
  EXPECT_EQ(sm.message(), "la carga sigue en el mecanismo");
  EXPECT_TRUE(sm.release_commanded());
}

TEST_F(PayloadTest, SensorLostWhileVerifyingIsNotReleased)
{
  start();
  confirm_and_step(good(5.0));
  sm.on_ack(true, 0);
  Inputs blind = good(8.0);
  blind.payload_valid = false;
  sm.step(blind);
  sm.step(blind);
  EXPECT_EQ(sm.outcome(), Outcome::NOT_RELEASED);
  EXPECT_EQ(sm.message(), "sin lectura del sensor de carga");
}

// SIM-06: PX4 veta la apertura aunque ROS 2 la ordene.
TEST_F(PayloadTest, DropGuardDenialGivesDeniedWithReason)
{
  start();
  confirm_and_step(good(5.0));
  sm.on_ack(false, 9);
  sm.step(good(5.1));
  EXPECT_EQ(sm.phase(), Phase::IDLE);
  EXPECT_EQ(sm.outcome(), Outcome::DENIED);
  EXPECT_EQ(sm.message(), "drop_guard rechazó la apertura: OUTSIDE_RADIUS");
}

TEST_F(PayloadTest, LateDenialWinsOverWaitingForSensor)
{
  start();
  confirm_and_step(good(5.0));
  sm.step(good(7.1));  // sin ack tras 2 s: pasa a VERIFYING
  EXPECT_EQ(sm.phase(), Phase::VERIFYING);
  sm.on_ack(false, 10);
  sm.step(good(7.2));
  EXPECT_EQ(sm.outcome(), Outcome::DENIED);
  EXPECT_EQ(sm.message(), "drop_guard rechazó la apertura: BELOW_BAND");
}

TEST_F(PayloadTest, NoAckButPayloadGoneCountsAsReleased)
{
  start();
  confirm_and_step(good(5.0));
  sm.step(good(7.0));
  Inputs released = good(7.1);
  released.payload_present = false;
  sm.step(released);
  EXPECT_EQ(sm.outcome(), Outcome::RELEASED);
  EXPECT_EQ(sm.message(), "carga liberada (sin ack de PX4)");
}

TEST_F(PayloadTest, NoAckAndPayloadStaysIsNotReleased)
{
  start();
  confirm_and_step(good(5.0));
  sm.step(good(7.0));
  sm.step(good(8.0));
  EXPECT_EQ(sm.outcome(), Outcome::NOT_RELEASED);
}

TEST_F(PayloadTest, AckOutsideReleaseIsIgnored)
{
  sm.on_ack(false, 2);   // IDLE
  start();
  sm.on_ack(false, 2);   // WAITING_CONFIRMATION
  confirm_and_step(good(5.0));
  sm.on_ack(true, 0);
  EXPECT_EQ(sm.step(good(5.1)).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::VERIFYING);
}

// Cada condición propia que falla al confirmar da DENIED sin ordenar la apertura.
struct BlockerCase {
  const char * name;
  void (*mutate)(Inputs &);
  const char * message;
};

class PayloadBlockerTest : public PayloadTest, public ::testing::WithParamInterface<BlockerCase> {};

TEST_P(PayloadBlockerTest, DeniesWithoutCommandingRelease)
{
  start();
  Inputs in = good(5.0);
  GetParam().mutate(in);
  EXPECT_EQ(confirm_and_step(in).type, CommandType::NONE);
  EXPECT_EQ(sm.phase(), Phase::IDLE);
  EXPECT_EQ(sm.outcome(), Outcome::DENIED);
  EXPECT_EQ(sm.message(), GetParam().message);
  EXPECT_FALSE(sm.release_commanded());
}

INSTANTIATE_TEST_SUITE_P(
  Blockers, PayloadBlockerTest,
  ::testing::Values(
    BlockerCase{"not_armed", [](Inputs & i) {i.armed = false;}, "el dron no está armado"},
    BlockerCase{"pos_invalid", [](Inputs & i) {i.pos_valid = false;}, "posición no válida o caducada"},
    BlockerCase{"pos_stale", [](Inputs & i) {i.pos_age_s = 1.001;}, "posición no válida o caducada"},
    BlockerCase{"pos_nan", [](Inputs & i) {i.pos.lat_deg = std::nan("");}, "posición no válida o caducada"},
    BlockerCase{"alt_nan", [](Inputs & i) {i.alt_amsl_m = std::nan("");}, "posición no válida o caducada"},
    BlockerCase{"eph_nan", [](Inputs & i) {i.eph_m = std::nan("");}, "posición no válida o caducada"},
    BlockerCase{"epv_negative", [](Inputs & i) {i.epv_m = -1.0;}, "posición no válida o caducada"},
    BlockerCase{"home_invalid", [](Inputs & i) {i.home_valid = false;}, "altura del punto de despegue no válida"},
    BlockerCase{"sensor_blind", [](Inputs & i) {i.payload_valid = false;}, "el sensor de carga no tiene lectura"},
    BlockerCase{"no_payload", [](Inputs & i) {i.payload_present = false;}, "no hay carga en el mecanismo"},
    BlockerCase{"dg_missing", [](Inputs & i) {i.dg_valid = false;}, "sin estado de drop_guard"},
    BlockerCase{"hash_mismatch", [](Inputs & i) {i.dg_zone_hash = kHash + 1;},
      "el hash de zona de drop_guard no coincide con la misión"},
    // d = 15 m al norte del centro (1 grado de latitud ≈ 111,2 km)
    BlockerCase{"outside_radius", [](Inputs & i) {i.pos.lat_deg = 43.6 + 25.0 / 111194.9;},
      "fuera del radio de la zona de suelta"},
    BlockerCase{"eph_shrinks_zone", [](Inputs & i) {i.pos.lat_deg = 43.6 + 15.0 / 111194.9; i.eph_m = 6.0;},
      "fuera del radio de la zona de suelta"},
    BlockerCase{"below_band", [](Inputs & i) {i.alt_amsl_m = 150.0 + 15.5;}, "fuera de la banda de altura de suelta"},
    BlockerCase{"above_band", [](Inputs & i) {i.alt_amsl_m = 150.0 + 29.5;}, "fuera de la banda de altura de suelta"},
    BlockerCase{"below_band_by_epv", [](Inputs & i) {i.alt_amsl_m = 150.0 + 16.0; i.epv_m = 1.5;},
      "fuera de la banda de altura de suelta"}),
  [](const ::testing::TestParamInfo<BlockerCase> & case_info) {return std::string(case_info.param.name);});

TEST_F(PayloadTest, EdgeOfZoneAndBandIsAllowed)
{
  start();
  Inputs in = good(5.0);
  in.pos.lat_deg = 43.6 + 14.0 / 111194.9;   // d ≈ 14 m, con eph 1 m → 15 m ≤ 20 m
  in.alt_amsl_m = 150.0 + 16.0;              // 16 m ≥ 15 + epv(1)
  EXPECT_EQ(confirm_and_step(in).type, CommandType::RELEASE);
}

TEST_F(PayloadTest, CancelWhileWaitingReturnsToIdleWithoutRelease)
{
  start();
  std::string msg;
  EXPECT_TRUE(sm.cancel(msg));
  EXPECT_EQ(sm.phase(), Phase::IDLE);
  EXPECT_EQ(sm.outcome(), Outcome::NONE);
  EXPECT_EQ(sm.step(good(1.0)).type, CommandType::NONE);
  EXPECT_FALSE(sm.release_commanded());
  start(2.0);  // se puede iniciar otra
}

TEST_F(PayloadTest, CancelAfterReleaseCommandedIsRejected)
{
  start();
  confirm_and_step(good(5.0));
  std::string msg;
  EXPECT_FALSE(sm.cancel(msg));
  EXPECT_EQ(sm.phase(), Phase::RELEASING);
  EXPECT_FALSE(sm.cancel(msg));
}

TEST_F(PayloadTest, CancelWithPendingConfirmationDiscardsIt)
{
  start();
  std::string msg;
  EXPECT_TRUE(sm.confirm(msg));
  EXPECT_TRUE(sm.cancel(msg));
  EXPECT_EQ(sm.step(good(1.0)).type, CommandType::NONE);
  start(2.0);
  EXPECT_EQ(sm.step(good(2.1)).type, CommandType::NONE);   // la confirmación anterior no sobrevive
}

TEST_F(PayloadTest, SecondDropAfterFinishStartsClean)
{
  start();
  confirm_and_step(good(5.0));
  sm.on_ack(false, 9);
  sm.step(good(5.1));
  ASSERT_EQ(sm.outcome(), Outcome::DENIED);

  start(20.0);
  EXPECT_EQ(sm.outcome(), Outcome::NONE);            // el resultado en curso se borra
  EXPECT_EQ(sm.last_outcome(), Outcome::DENIED);     // el último se conserva para PayloadState
  EXPECT_FALSE(sm.release_commanded());
  // El ack DENIED de la suelta anterior no contamina la nueva
  EXPECT_EQ(confirm_and_step(good(21.0)).type, CommandType::RELEASE);
  sm.on_ack(true, 0);
  sm.step(good(21.1));
  EXPECT_EQ(sm.phase(), Phase::VERIFYING);
}

TEST(PayloadNames, ReasonsAndEnums)
{
  EXPECT_STREQ(drone_payload::drop_guard_reason_name(0), "OK");
  EXPECT_STREQ(drone_payload::drop_guard_reason_name(11), "ABOVE_BAND");
  EXPECT_STREQ(drone_payload::drop_guard_reason_name(99), "DESCONOCIDO");
  EXPECT_STREQ(drone_payload::drop_guard_reason_name(-1), "DESCONOCIDO");
  EXPECT_STREQ(drone_payload::to_string(Phase::VERIFYING), "VERIFYING");
  EXPECT_STREQ(drone_payload::to_string(Outcome::NOT_RELEASED), "NOT_RELEASED");
}
