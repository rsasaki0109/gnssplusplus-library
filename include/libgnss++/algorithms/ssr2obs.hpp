#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/observation.hpp>

#include <vector>

namespace libgnss::ssr2obs {

/// \brief Turn raw RINEX-style observations into **SSR/OSR-adjusted** observations.
///
/// \par Plain language (English)
/// For each satellite/signal, subtract the same correction *meters* that CLAS OSR PPP
/// already subtracts before forming residuals. After this call, `pseudorange` is still
/// in meters and `carrier_phase` is still in cycles, but both represent
/// **“observable − OSR correction”**.
///
/// \par 日本語（要点）
/// 各衛星・各信号について、CLAS OSR PPP と同じ補正量（メートル換算）を
/// **仮想距離からそのまま引き**、**搬送波（サイクル）からは 波長で割った分だけ引く**。
/// いわゆる「補正を観測値に織り込んだ」状態にします。ネットワーク補間した
/// **仮想基準局の座標そのもの**は作りません（VRS 全体の再現ではありません）。
///
/// \par CLASLIB naming
/// Closest built-in analogue to `SSR2OBS` **at the rover**: same math as
/// `selectAppliedOsrCorrections` inside `ppp_clas.cpp`.
///
/// \par Typical call chain
/// `SSRProducts` → `prepareClasEpochContext` / `ssr2osr::computeOSR` → **this function**,
/// or `buildSsrCorrectedObservationsFromClasContext` if you already hold `CLASEpochContext`.
ObservationData buildSsrCorrectedObservations(
    const ObservationData& raw,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config);

/// \brief Same as `buildSsrCorrectedObservations`, but reads OSR rows from `epoch_context`.
/// \par 日本語: `CLASEpochContext` に入っている `osr_corrections` をそのまま使う短縮形。
inline ObservationData buildSsrCorrectedObservationsFromClasContext(
    const ObservationData& raw,
    const CLASEpochContext& epoch_context,
    const ppp_shared::PPPConfig& config) {
    return buildSsrCorrectedObservations(
        raw, epoch_context.osr_corrections, config);
}

}  // namespace libgnss::ssr2obs
