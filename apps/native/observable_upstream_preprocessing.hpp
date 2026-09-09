#pragma once

// Compatibility wrapper for the standalone smartphone PDC application.
// The raw-observable contract is library-owned so the native FGO graph and
// the dedicated PDC executable cannot drift into different equations.
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
