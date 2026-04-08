#pragma once

/// \file ssr2osr.hpp
/// \par Purpose
/// One-line include for the **SSR → OSR** stage: build per-satellite `OSRCorrection`
/// bundles (CLASLIB: `cssr2osr.c` / “SSR2OSR” in docs).
///
/// \par 日本語
/// **「補正データから、各衛星の観測空間用の補正セットを作る」**段階の入り口です。\
/// 中身は `ppp_osr.hpp` と同じで、`namespace libgnss::ssr2osr` で `computeOSR` などを
/// 別名検索しやすくしただけです。\
/// その次に **生の観測値へ織り込む**には `ssr2obs.hpp`（`SSR2OBS` に近い）を使います。
#include <libgnss++/algorithms/ppp_osr.hpp>
