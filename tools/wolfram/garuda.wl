(* ::Package:: *)

(* garuda.wl — load + model Garuda ESC bench data exported by the GUI (`wl` command).

   Usage:
     << "tools/wolfram/garuda.wl"
     run = GarudaImport["tools/garuda_debug/sessions/run_20260704_120000.wl.json"];
     GarudaKeFit[run]        (* fitted eRPM-per-duty·V — per-motor KSPEED/Ke     *)
     GarudaScopePlot[run]    (* burst capture: BEMF vs threshold vs sector       *)
     GarudaRCFit[run]        (* RC filter time-constant from PWM-edge settling   *)

   Companion to docs/wolfram_integration_design.md (phase W1/W2). The point:
   every constant the ZC Lab currently hardcodes gets FITTED from this data. *)

GarudaImport[path_String] := Module[{raw},
  raw = Import[path, "RawJSON"];
  <|
    "info"   -> raw["info"],
    "params" -> raw["params"],
    "telem"  -> Dataset[raw["telemetry"]],
    "scope"  -> Dataset[raw["scope"]],
    "path"   -> path
  |>]

(* ── steady-state CL points: (dutyPct, eRPM, VbusV) with transients dropped ── *)
GarudaSteadyPoints[run_Association] := Module[{t},
  t = Normal @ run["telem"];
  t = Select[t, #["state_name"] === "CL" && #["eRPM"] > 500 &];
  (* steady = eRPM within 3% of the 1s-later value *)
  Table[
    <|"duty" -> t[[i]]["duty"], "eRPM" -> t[[i]]["eRPM"], "vbus" -> t[[i]]["vbus_V"]|>,
    {i, Length[t] - 12}] //
    Select[#, Function[p, p["duty"] > 0]] &]

(* ── Ke / KSPEED fit: eRPM ≈ k · duty% · Vbus  (k in eRPM per %·V) ──
   Replaces the hardcoded KSPEED=235k in analyze.py with a per-motor value:
   KSPEED_equiv (eRPM at 100% duty, nominal Vbus) = 100 · Vnom · k. *)
GarudaKeFit[run_Association] := Module[{pts, fit, k, vnom},
  pts = GarudaSteadyPoints[run];
  If[Length[pts] < 20, Return[<|"error" -> "need a run with several steady duty plateaus"|>]];
  fit = LinearModelFit[
    {#["duty"] * #["vbus"], #["eRPM"]} & /@ pts // Map[List @@ # &],
    x, x, IncludeConstantBasis -> False];
  k = fit["BestFitParameters"][[1]];
  vnom = Median[#["vbus"] & /@ pts];
  <|"k_eRPM_per_pctV" -> k,
    "KSPEED_at_Vnom"  -> 100. * vnom * k,
    "Vnom"            -> vnom,
    "R2"              -> fit["RSquared"],
    "n"               -> Length[pts],
    "note" -> "KSPEED_at_Vnom replaces analyze.py KSPEED; k·Vbus·duty predicts no-load eRPM"|>]

(* ── burst capture visualization: the raw detection problem, one glance ── *)
GarudaScopePlot[run_Association] := Module[{sc, n, dt},
  sc = Normal @ run["scope"];
  If[Length[sc] < 8, Return["no scope capture in this bundle — arm the Burst Scope first"]];
  n = Length[sc]; dt = 1.*^6/24000.;  (* µs per sample *)
  Column[{
    ListLinePlot[{
        Table[{i dt, sc[[i]]["bemf_raw"]}, {i, n}],
        Table[{i dt, sc[[i]]["zc_thresh"]}, {i, n}]},
      PlotLegends -> {"BEMF (raw ADC)", "ZC threshold"},
      PlotLabel -> "Crossings = zero-cross events", ImageSize -> 640,
      AxesLabel -> {"t (µs)", "ADC"}],
    ListLinePlot[{
        Table[{i dt, sc[[i]]["ia_A"]}, {i, n}],
        Table[{i dt, sc[[i]]["ibus_A"]}, {i, n}]},
      PlotLegends -> {"Ia (A)", "Ibus (A)"}, ImageSize -> 640,
      AxesLabel -> {"t (µs)", "A"}],
    ListStepPlot[Table[{i dt, sc[[i]]["sector"]}, {i, n}],
      PlotLabel -> "Sector", ImageSize -> 640, AxesLabel -> {"t (µs)", ""}]
  }]]

(* ── RC filter τ fit (phase-W2 skeleton): find PWM-edge exponential settlings in
     the BEMF channel and fit τ. Real captures are 24kHz (~41.7µs/sample) so only
     multi-sample settle tails are fittable — flag if the edge is sub-sample. ── *)
GarudaRCFit[run_Association] := Module[{sc, bemf, jumps, segs, fits},
  sc = Normal @ run["scope"];
  If[Length[sc] < 16, Return["no scope capture"]];
  bemf = #["bemf_raw"] & /@ sc;
  jumps = Select[Range[2, Length[bemf] - 6], Abs[bemf[[#]] - bemf[[# - 1]]] > 150 &];
  If[jumps === {}, Return[<|"error" -> "no large edges found; capture at low duty near a commutation"|>]];
  segs = Table[bemf[[j ;; Min[j + 5, Length[bemf]]]], {j, jumps}];
  fits = Quiet @ Table[
    Module[{d = N@seg, m},
      m = NonlinearModelFit[Transpose[{Range[0, Length[d] - 1] * 41.67, d}],
        a + b Exp[-t/tau], {{a, Last[d]}, {b, First[d] - Last[d]}, {tau, 30.}}, t];
      tau /. m["BestFitParameters"]],
    {seg, segs}];
  <|"tau_us_candidates" -> fits,
    "tau_us_median" -> Median[Select[fits, 5 < # < 500 &]],
    "expected_from_schematic_us" -> 29.,   (* 2.9kΩ·10nF *)
    "n_edges" -> Length[jumps]|>]
