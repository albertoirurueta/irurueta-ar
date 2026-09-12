# Implementation Plan

## Task summary

Source: GitHub issue #18

The Bibliography section of `docs/modules/ROOT/pages/reference.adoc` lists 43 citations (books and papers) as
plain text, with no outbound links. Issue #18 asks to add, per entry, a link to:
- the book's official editorial/publisher page, or
- the official site where the paper can be downloaded (its DOI page, or the publisher/venue's page when no DOI
  exists).

The issue also requires this work to originate on `release_1.6.0` (already satisfied — branch `feature/18` was
created from `release_1.6.0`) so the change appears in the docs for the most recent version, then be merged into
`master` and `develop`. That merge sequencing happens after this PR is reviewed/merged and is outside the scope of
this plan's tasks.

**Approach chosen**: every citation's title is wrapped in an AsciiDoc inline link
(`"https://url[Title Text]"`), preserving the existing citation text and anchors (`[[ref-xxx]]`) exactly —
only the title substring becomes a hyperlink. This is the least invasive change: anchors used by `xref:` links
elsewhere in the docs site are untouched, and the citation text/formatting is otherwise unchanged.

Each URL below was researched against the actual title/authors/venue (DOI preferred, then IEEE Xplore/ACM/
Springer/publisher page, then a conference/author page as a last resort). One entry (`ref-alvarez-1999`) has no
credible official source at all (an obscure 1999 Spanish national conference proceedings, no DOI/publisher
record found) — leave that citation as plain text, unlinked, rather than link to an unofficial mirror
(ResearchGate/Academia.edu). This was a judgment call per the issue's own instruction to link only to *official*
sources.

## Current code state

- `docs/modules/ROOT/pages/reference.adoc` — Antora page (component root `docs/`, per `docs/antora.yml`).
  - Lines 1–46: report links + Attachments table (unaffected by this change).
  - Lines 48–159: `== Bibliography` section — a single AsciiDoc ordered list (`. [[anchor]] citation text`),
    43 items, each with a unique `[[ref-xxx]]` anchor referenced by `<<ref-xxx>>` elsewhere in the docs site.
    No item currently has a hyperlink.
- No Java source is affected — this is a documentation-only change (AsciiDoc), so no task below carries a
  `java`/`dotnet`/`database` language tag; a downstream execution skill should implement these directly rather
  than dispatch to a language-specific code skill.
- Antora docs build via the `iru-build-docs` skill (or `npx antora docs/antora-playbook.yml` under `docs/`,
  per that skill) — this is the mechanism to verify no AsciiDoc syntax was broken by the edits (unbalanced
  `[...]`/`"..."`, a malformed link macro, etc.).

## Implementation steps

### Group 1 — Add links to every bibliography entry (Parallelizable: yes — trivially, one task)

- [x] Task 1. Add an official link to each Bibliography entry in `docs/modules/ROOT/pages/reference.adoc`
  by wrapping the citation's title in an AsciiDoc link macro `https://url[Title text]`, leaving everything
  else on the line unchanged. Apply exactly these 43 edits (anchor → URL to use):
  **Done** — file touched: `docs/modules/ROOT/pages/reference.adoc`. All 48 linked sub-tasks (1.1-1.37, 1.39-1.49)
  applied exactly as specified; `ref-alvarez-1999` (Task 1.38) left unlinked as instructed. Documentation-only
  change — no tests/coverage/license headers apply to this `.adoc` file; AsciiDoc build verification is Group 2.

  - [x] Task 1.1. `ref-nyquist-1924` → `https://doi.org/10.1002/j.1538-7305.1924.tb01361.x`
  - [x] Task 1.2. `ref-scharstein-szeliski-2002` → `https://doi.org/10.1023/A:1014573219977`
  - [x] Task 1.3. `ref-birchfield-tomasi-1998` → `https://doi.org/10.1109/34.677269`
  - [x] Task 1.4. `ref-scharstein-szeliski-1998` → `https://doi.org/10.1023/A:1008015117424`
  - [x] Task 1.5. `ref-cyganek-siebert-2009` → `https://www.wiley.com/en-us/An+Introduction+to+3D+Computer+Vision+Techniques+and+Algorithms-p-9781119964476` (book, Wiley product page)
  - [x] Task 1.6. `ref-banks-corke-2001` → `https://doi.org/10.1177/02783640122067525`
  - [x] Task 1.7. `ref-pollefeys-2000` → `https://cvg.ethz.ch/lectures/Computer-Vision/assets/tutorial.pdf` (no DOI/publisher exists for these ECCV2000 tutorial notes; author's own hosted copy is the closest official source)
  - [x] Task 1.8. `ref-hartley-zisserman-2003` → `https://www.cambridge.org/9780521540513` (book, Cambridge University Press product page)
  - [x] Task 1.9. `ref-faugeras-1993` → `https://mitpress.mit.edu/9780262061582/three-dimensional-computer-vision/` (book, MIT Press)
  - [x] Task 1.10. `ref-faugeras-luong-2004` → `https://mitpress.mit.edu/9780262562041/the-geometry-of-multiple-images/` (book, MIT Press)
  - [x] Task 1.11. `ref-mordohai-medioni-2006` → `https://doi.org/10.1109/TPAMI.2006.129`
  - [x] Task 1.12. `ref-yoon-kweon-2006` → `https://doi.org/10.1109/TPAMI.2006.70`
  - [x] Task 1.13. `ref-taguchi-2008` → `https://ieeexplore.ieee.org/document/4587691/` (no confirmed resolvable DOI found; official IEEE Xplore document page used directly)
  - [x] Task 1.14. `ref-kanade-okutomi-1994` → `https://doi.org/10.1109/34.310690`
  - [x] Task 1.15. `ref-wang-2006` → `https://doi.org/10.1109/3DPVT.2006.75`
  - [x] Task 1.16. `ref-tombari-2007` → `https://doi.org/10.1007/978-3-540-77129-6_38`
  - [x] Task 1.17. `ref-yin-cooperstock-2004` → `http://wscg.zcu.cz/wscg2004/wscg2004.htm` (no DOI/publisher record for this WSCG 2004 paper; official conference page used as closest official source)
  - [x] Task 1.18. `ref-hirschmuller-2006` → `https://doi.org/10.1109/CVPR.2006.294`
  - [x] Task 1.19. `ref-harris-stephens-1988` → `https://doi.org/10.5244/C.2.23`
  - [x] Task 1.20. `ref-lowe-1999` → `https://doi.org/10.1109/ICCV.1999.790410`
  - [x] Task 1.21. `ref-memin-perez-2002` → `https://doi.org/10.1023/A:1013539930159`
  - [x] Task 1.22. `ref-wang-zheng-2008` → `https://doi.org/10.1109/CVPR.2008.4587456`
  - [x] Task 1.23. `ref-brox-2004` → `https://doi.org/10.1007/978-3-540-24673-2_3`
  - [x] Task 1.24. `ref-mainberger-2008` → `https://doi.org/10.1007/978-3-540-69812-8_62`
  - [x] Task 1.25. `ref-kang-2001` → `https://www.microsoft.com/en-us/research/publication/handling-occlusions-in-dense-multi-view-stereo/` (no IEEE Xplore/DOI record found; official Microsoft Research publication page used instead)
  - [x] Task 1.26. `ref-black-1991` → `https://doi.org/10.1109/CVPR.1991.139705`
  - [x] Task 1.27. `ref-bhusnurmath-taylor-2008` → `https://repository.upenn.edu/server/api/core/bitstreams/af1d8e6a-328e-40ef-8dfc-08ed283d6be2/content` (no IEEE Xplore/DOI page found for this 3DPVT 2008 paper; author's institutional repository copy used as the closest official source — note the citation's stated venue, "European Conference on Computer Vision", appears to actually be 3DPVT 2008, but this task only adds a link, it does not correct the citation text)
  - [x] Task 1.28. `ref-slesareva-2005` → `https://doi.org/10.1007/11550518_5`
  - [x] Task 1.29. `ref-ohta-kanade-1985` → `https://doi.org/10.1109/TPAMI.1985.4767639`
  - [x] Task 1.30. `ref-csurka-1996` → `https://doi.org/10.1006/cviu.1997.0531`
  - [x] Task 1.31. `ref-luong-faugeras-1995` → `https://doi.org/10.1007/BF00127818`
  - [x] Task 1.32. `ref-zhang-1998` → `https://doi.org/10.1023/A:1007941100561`
  - [x] Task 1.33. `ref-atzpadin-2004` → `https://ieeexplore.ieee.org/document/1273542/` (no confirmed resolvable DOI found; official IEEE Xplore document page used directly)
  - [x] Task 1.34. `ref-press-2007` → `https://www.cambridge.org/numericalrecipes` (book, Cambridge University Press)
  - [x] Task 1.35. `ref-cormen-2001` → `https://mitpress.mit.edu/9780262531962/introduction-to-algorithms/` (book, MIT Press, confirmed as the 2nd edition ISBN matching the citation)
  - [x] Task 1.36. `ref-george-tjahjadi-1999` → `https://ieeexplore.ieee.org/document/817210/` (no confirmed resolvable DOI found; official IEEE Xplore document page used directly)
  - [x] Task 1.37. `ref-alvarez-2000` → `https://doi.org/10.1023/A:1008170101536`
  - [x] Task 1.38. `ref-alvarez-1999` → **no link** — leave this citation exactly as-is (plain text, unlinked). No DOI, publisher page, or official record could be found for this CEDYA XVI (1999) proceedings paper; only unofficial mirrors (ResearchGate/Academia.edu) exist, and the issue asks for links to *official* sources only.
  - [x] Task 1.39. `ref-bergen-1992` → `https://doi.org/10.1007/3-540-55426-2_27`
  - [x] Task 1.40. `ref-zitnick-2004` → `https://doi.org/10.1145/1186562.1015766`
  - [x] Task 1.41. `ref-pollefeys-koch-gool-1999` → `https://doi.org/10.1109/ICCV.1999.791262`
  - [x] Task 1.42. `ref-hartley-1993` → `https://users.cecs.anu.edu.au/~hartley/Papers/cheiral/cheiral.pdf` (no DOI/publisher page exists for this 1993 DARPA/ARPA workshop paper; author's own (Richard Hartley, ANU) hosted copy used as the closest official source)
  - [x] Task 1.43. `ref-laveau-faugeras-1996` → `https://doi.org/10.1007/BFb0015531`
  - [x] Task 1.44. `ref-horn-schunck-1981` → `https://doi.org/10.1016/0004-3702(81)90024-2`
  - [x] Task 1.45. `ref-perona-malik-1990` → `https://doi.org/10.1109/34.56205`
  - [x] Task 1.46. `ref-hornung-kobbelt-2006` → `https://doi.org/10.2312/SGP/SGP06/041-050`
  - [x] Task 1.47. `ref-kazhdan-2006` → `https://doi.org/10.2312/SGP/SGP06/061-070`
  - [x] Task 1.48. `ref-merrell-2007` → `https://doi.org/10.1109/ICCV.2007.4408984`
  - [x] Task 1.49. `ref-fischler-bolles-1981` → `https://doi.org/10.1145/358669.358692`

  Example of the transformation (Task 1.1, `ref-nyquist-1924`):
  ```
  Before:
  . [[ref-nyquist-1924]] H. Nyquist. "Certain factors affecting telegraph speed". _Bell System Technical Journal_,
    324-346, 1924.

  After:
  . [[ref-nyquist-1924]] H. Nyquist. "https://doi.org/10.1002/j.1538-7305.1924.tb01361.x[Certain factors affecting
    telegraph speed]". _Bell System Technical Journal_, 324-346, 1924.
  ```
  Do not alter anchors, author names, venue/publisher text, years, or line-wrapping conventions already used in
  the file — the only change per entry is wrapping the title in the link macro (Task 1.38 excepted, which gets no
  change at all).

### Group 2 — Verify the docs build (Parallelizable: yes — trivially, one task; depends on Group 1 being complete)

- [x] Task 2. Verify `docs/modules/ROOT/pages/reference.adoc` still builds cleanly after all 42 edits, since a
  malformed AsciiDoc link macro (unbalanced `[`/`]` or `"`) would break the page silently or fail the build.
  Delegate this to a sub-agent so build output doesn't consume the main context window:
  ```
  Agent({
    description: "Build Antora docs to verify reference.adoc",
    subagent_type: "iru-gate-runner",
    prompt: "Invoke Skill({skill: \"iru-build-docs\"}) to build this repository's Antora documentation site. Report back only whether the build succeeded or failed, and if it failed, the specific error and which file/line it points to."
  })
  ```
  **Done** — `iru-gate-runner` reported the Antora build succeeded (built site at
  `docs/build/site/index.html`), no errors. No malformed link macros found.
  - [x] Task 2.1. If the build fails, locate and fix the malformed link macro(s) in `reference.adoc` and re-run
    the build until it succeeds.
    **N/A** — build succeeded on the first run; no fix needed.
  - [x] Task 2.2. Once the build succeeds, spot-check a handful of rendered links in the built HTML output
    (e.g. `docs/build/site/irurueta-ar/<version>/reference.html`) to confirm the title text is a clickable link
    pointing at the intended URL, not literal `https://...[Title]` text (which would indicate a macro syntax
    mistake even if the build itself didn't fail).
    **Done** — inspected `docs/build/site/irurueta-ar/1.6.0/reference.html`: found 53 rendered `<a href="https://...">`
    links in the page, spot-checked `ref-nyquist-1924` (DOI link, correct title text anchor), `ref-cyganek-siebert-2009`
    (Wiley book page, correct title text anchor), `ref-faugeras-1993` (MIT Press page, correct title text anchor),
    and confirmed `ref-alvarez-1999` correctly remains plain unlinked text. A repo-wide grep for the literal broken
    pattern `https://...[` found zero matches — no macro syntax mistakes.
