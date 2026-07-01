(* Single source of truth for per-motor constants. Values copied verbatim from
   spec 11a (docs/superpowers/specs/2026-07-01-pll-sensing-chain-wolfram-model-design.md).
   Rs stored PER PHASE (VEX datasheet 0.44 ohm is phase-to-phase => /2). *)

MotorParams = <|
  "VEX" -> <|
    "shape" -> "trap", "KV" -> 4000, "PP" -> 6,
    "KeUvSRad" -> 230, "RsOhm" -> 0.22, "LsH" -> 9.*^-6,
    "tauSec" -> 30.*^-6,
    "advMax" -> 22, "advStartErpm" -> 45000, "advFullErpm" -> 85000
  |>,
  "2810" -> <|
    "shape" -> "trap", "KV" -> 1350, "PP" -> 7,
    "KeUvSRad" -> 583, "RsOhm" -> 0.022, "LsH" -> 10.*^-6,
    "tauSec" -> 30.*^-6,
    "advMax" -> 22, "advStartErpm" -> 45000, "advFullErpm" -> 85000
  |>,
  "Hurst300" -> <|
    "shape" -> "sine", "KV" -> Missing["NotApplicable"], "PP" -> 5,
    "KeUvSRad" -> Missing["Provided as V*s/krpm peak = 7.34"],
    "RsOhm" -> 0.285, "LsH" -> 287.*^-6,
    "tauSec" -> 30.*^-6,
    "advMax" -> 22, "advStartErpm" -> 45000, "advFullErpm" -> 85000
  |>
|>;

omegaElec[erpm_] := 2. Pi (erpm/60.);

ExportModelicaParams[motor_String, path_String] := Module[{p = MotorParams[motor]},
  Export[path,
    StringJoin[
      "record ", motor, "Params\n",
      "  parameter Integer PP = ", ToString[p["PP"]], ";\n",
      "  parameter Real KeUvSRad = ", ToString[p["KeUvSRad"]], ";\n",
      "  parameter Real RsOhm = ", ToString[p["RsOhm"]], ";\n",
      "  parameter Real LsH = ", ToString[CForm[p["LsH"]]], ";\n",
      "  parameter Real tauSec = ", ToString[CForm[p["tauSec"]]], ";\n",
      "end ", motor, "Params;\n"
    ], "Text"]
];
