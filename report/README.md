# MOTH group report (LaTeX)

Source for the AME 101 Monarch of the Hill group report, "Win the Ground Game, Win the Hill" (Team 2). This replaces the Google Doc "MOTH Group Report" as the source of truth from Oct 26, 2026.

## Open in Overleaf

1. Download `moth-report-overleaf.zip`, or zip the contents of this `report/` folder yourself so that `main.tex` sits at the top level of the zip.
2. In Overleaf: **New Project → Upload Project**, and choose the zip.
3. The compiler is pdfLaTeX (Overleaf's default) and the main document is `main.tex`. Both are the defaults, so it should compile with no settings changed.

## Layout

| Path | What it holds |
| --- | --- |
| `main.tex` | Preamble (fonts, page, tables, placeholder macros) and the list of sections |
| `sections/` | One file per section: front matter (title page, abstract, contents, nomenclature), Sections 1 to 12, references, appendices |
| `figures/` | Every figure. `fig04_gear_ratio.pdf` and `fig11_pitch.pdf` are vector plots |
| `figures/src/` | matplotlib scripts that regenerate the two vector plots (`python fig04_gear_ratio.py`, `python fig11_pitch.py`, run from this folder) |
| `code/` | Photon 2 firmware listed in Appendix A |

## Conventions

- **Placeholders.** `\tbd{...}` prints yellow-highlighted bracketed text for anything still to be confirmed or filled in. Inside `\tbd{}`, wrap math and units in `\mbox{}` (for example `\tbd{two \mbox{\qty{100}{nF}} capacitors}`), or the highlighter fails to compile. `\placeholderfig{height}{text}` draws a dashed yellow box where a figure still has to go. Setting `\showtbdfalse` in `main.tex` turns the highlight off.
- **Cross-references.** Figures, tables, equations, sections and citations are numbered automatically. Refer to them by label: `Fig.~\ref{fig:gear}`, `Table~\ref{tab:current}`, `Eq.~\eqref{eq:climb}`, `Section~\ref{sec:couplers}`, `\cite{tb6612}`.
- **References** are ASME numbered style, listed in `sections/references.tex` in order of first citation. Add a `\bibitem{key}` where the source is first cited and the numbers update themselves.
- **Units** use `siunitx`: `\qty{1.524}{m}`, `\ang{33.4}`, `\unit{N.m}`.
