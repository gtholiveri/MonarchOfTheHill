# MOTH group report (LaTeX)

Source for the AME 101 Monarch of the Hill group report, "Win the Ground Game, Win the Hill" (Team 2). This folder has been the source of truth since Oct 26, 2026. The Google Doc "MOTH Group Report" is frozen, so don't edit it.

**Teammates: you never need git or a LaTeX install.** Edit in Overleaf in your browser. It works like Google Docs: everyone edits the same copy live, and the PDF rebuilds when you hit Recompile.

## 1. Get in

1. Gabriel invites you from Overleaf's **Share** button with "Can edit". Accept the email invite (make a free Overleaf account with that same email if you don't have one).
2. Open the project. The file tree is on the left, the source in the middle and the PDF on the right.
3. Click **Recompile** to see the PDF. If it fails with "no main document", open **Menu → Main document** and pick `report/main.tex`.

## 2. Find your section

Everything you'll edit is in `report/sections/`. One file per section:

| Report section | File |
| --- | --- |
| Title page, abstract, nomenclature | `00-front-matter.tex` |
| 1 Introduction | `01-introduction.tex` |
| 2 Design concepts and selection | `02-concepts.tex` |
| 3 Drivetrain | `03-drivetrain.tex` |
| 4 Traction | `04-traction.tex` |
| 5 Chassis, wedge, forks and side guards | `05-chassis.tex` |
| 6 Electronics and power | `06-electronics.tex` |
| 7 Sensing the crest | `07-sensing.tex` |
| 8 Autonomous control | `08-control.tex` |
| 9 Testing summary | `09-testing.tex` |
| 10 Contest results | `10-results.tex` |
| 11 Lessons learned | `11-lessons.tex` |
| 12 Conclusion | `12-conclusion.tex` |
| References | `references.tex` |
| Appendices (firmware, raw data) | `appendices.tex` |

Images go in `report/figures/`. Leave `report/main.tex` alone (it's the page setup and fonts).

Tip: click a spot in the PDF and Overleaf jumps to that line in the source.

## 3. Syncing with GitHub (pull before, push after)

Gabriel and Claude also edit these files through GitHub, so Overleaf has to be synced with it. If you see **Menu → GitHub** (or **Integrations → GitHub** in the left sidebar):

- **Before you start editing:** open it and click **Pull GitHub changes into Overleaf**.
- **When you're done:** open it again and click **Push Overleaf changes to GitHub**.

If you don't see a GitHub option, skip this and tell Gabriel when you're done. He syncs for everyone.

## 4. Avoiding conflicts

Edits between teammates inside Overleaf never conflict, because it's one live copy. Conflicts only happen when the same lines change in Overleaf and on GitHub between syncs. To keep that from happening:

- Pull first (step 3), and tell the group chat which section you're in.
- Change the sentences you mean to change. Don't paste a whole section back in from an older copy (the Google Doc, a download, a chat), because that silently undoes everyone's later fixes.
- Push soon after you finish, not hours later.

If Overleaf says there's a conflict, or that it pushed to a branch named `overleaf-<date>`, **don't try to fix it.** Your work is safe on that branch. Tell Gabriel and he'll merge it.

Made a mess? **History** (top right) shows every change by name and can restore any earlier version.

## 5. LaTeX you'll actually hit

- **Paragraphs.** A blank line starts a new paragraph. Single line breaks are ignored.
- **Bold, italic.** `\textbf{bold}`, `\emph{italic}`.
- **Special characters.** `% & $ # _` mean something to LaTeX. To print them, type `\%`, `\&`, `\$`, `\#`, `\_`.
- **Units.** Always `\qty{1.4}{A}`, `\qty{20.5}{MPa}`, `\qty{100}{\percent}`. Angles are `\ang{33.4}`. Don't type a ° symbol.
- **Superscripts.** Don't use `\textsuperscript` (our Times font breaks it). Write "1st" plainly, and use `\unit{m^2}` or math like `$x^2$` for exponents.
- **Math.** Inline math goes between dollar signs: `$\mu_s$`.
- **Cross-references.** Figures, tables, equations and citations number themselves. Refer to them by label: `Fig.~\ref{fig:tracks}`, `Table~\ref{tab:friction}`, `Eq.~\eqref{eq:traction}`, `Section~\ref{sec:drivetrain}`. Never type a number by hand.
- **Citations.** `\cite{tb6612}` cites a source by its key. To add a new source, add a `\bibitem{yourkey} ...` entry to `references.tex`, copying the format of the ones already there. Our style is ASME (numbered), never APA.
- **Yellow placeholders.** `\tbd{...}` is anything still unconfirmed or unfilled; it prints highlighted in yellow. Replace the whole `\tbd{...}` with the real text once you know it. Inside `\tbd{}`, wrap any math or `\qty`/`\ang` in `\mbox{}`, like `\tbd{about \mbox{\qty{1.4}{A}}}`, and don't write `e.g.\ ` inside it, or the PDF won't compile.
- **Figures.** Upload the image into `report/figures/` (the upload icon at the top of the file tree), then paste this where it should go and fill it in:

  ```latex
  \begin{figure}[htbp]
    \centering
    \includegraphics[width=0.6\textwidth]{my_photo.jpg}
    \caption{One sentence saying what the reader should see.}
    \label{fig:myphoto}
  \end{figure}
  ```

  Then refer to it in the text as `Fig.~\ref{fig:myphoto}`. A dashed yellow box from `\placeholderfig{...}` marks a figure still to be made; replace that line with `\includegraphics`.

**When it won't compile:** click the red error count by Recompile. The first error gives a line number, and the problem is usually on that line: an unescaped `%` or `&`, or a missing `}`. Undo your last change if you can't see it.

## 6. Leaving comments for each other

- **Overleaf comment (preferred).** Highlight text, click **Add comment** in the review panel, type your note. Everyone sees it on the right and can reply or resolve it. Overleaf comments don't sync to GitHub, so if Claude needs to act on one, tell Gabriel.
- **Comment in the source.** A line that starts with `%` never prints, e.g. `% Efe: need the real number from Tuesday's test`. Start it with your name. These do sync to GitHub, so everyone (and Claude) can see them.

## Reference (for Gabriel and Claude)

- Build: pdfLaTeX, main document `report/main.tex`, sections pulled in with `\input`. Fonts are newtx (Times) 12 pt; tables use booktabs; units siunitx.
- `figures/src/` holds the matplotlib scripts that regenerate the vector plots `fig04_gear_ratio.pdf` and `fig11_pitch.pdf` (run from that folder).
- `\showtbdtrue` in `main.tex` turns the yellow highlight on; switch to `\showtbdfalse` only once no `\tbd{}` remain.
- References are listed in `references.tex` in order of first citation; the numbers follow that order.
- Overleaf pushes "Updates from Overleaf" merge commits straight to `master`, so fetch and rebase on `origin/master` before every push from git.
