# v0.2.0 growth launch runbook

This is a maintainer runbook for a small, human-reviewed outreach sequence.
The value is a reproducible measurement trail from the current public baseline
toward 200 GitHub stars; it is not an instruction to automate posting,
coordinate votes, or publish private traffic data.

## Baseline and evidence

The read-only baseline supplied on **2026-08-12 JST** is:

| Public metric | Baseline | Target/delta |
|---|---:|---:|
| GitHub stars | 183 | 200 / gap 17 |
| Forks | 21 | context only |
| Watchers | 7 | context only |

Source: the maintainer's read-only GitHub repository/release metadata snapshot
for [`rsasaki0109/gnssplusplus-library`](https://github.com/rsasaki0109/gnssplusplus-library)
and the [v0.2.0 release](https://github.com/rsasaki0109/gnssplusplus-library/releases/tag/v0.2.0),
captured on that date. The release assets were each at one download in the
supplied evidence. The GitHub traffic API was also checked with owner access,
but its views/clones values are intentionally not written here: keep those
owner-only values in ignored local snapshots only.

The measurement interpretation follows GitHub's [traffic REST API
reference](https://docs.github.com/en/rest/metrics/traffic) and [repository
traffic guidance](https://docs.github.com/en/repositories/viewing-activity-and-data-for-your-repository/viewing-traffic-to-a-repository).
Traffic details require repository push/admin visibility and cover a rolling
14-day window. The script records an exact UTC `captured_at` in each local
snapshot; the baseline above is the supplied date-level evidence, not a claim
that an unseen private timestamp is reproducible.

## Preflight gate

Complete this list before drafting a post. Record the check date and URLs in
the local launch notes, not in a public post:

- Confirm the `v0.2.0` release, assets, [self-contained offline demo](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/self_contained_demo.md), and the three [use-case routes](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases.md) are live.
- Confirm the [RTKLIB migration guide](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/rtklib_migration.md), [ROS2 guide](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/ros2.md), and [QZSS L6/CLAS/MADOCA guide](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/qzss_l6.md) still contain runnable commands and boundary notes.
- Verify every benchmark or accuracy statement against the [validation guide](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/validation.md); prefer the demo and measured artifact contracts over a broad performance claim.
- Disclose maintainer affiliation in every channel that requires or benefits from it. Do not present the maintainer as an independent user.
- Read the current rules for the selected channel immediately before posting. The [Show HN guidance](https://news.ycombinator.com/showhn.html) and [ROS Discourse release-announcement guideline](https://discourse.ros.org/t/community-software-release-announcements-guideline/1305) are references, not permanent permission.
- Use one post per relevant channel, with a distinct time window and a channel-specific purpose. Do not send a cross-post blast.
- Do not ask for stars, upvotes, likes, coordinated votes, or reciprocal engagement. Do not use automated posting, browser automation, or a bot to answer replies.

The supplied referrer evidence included GitHub, Google, LinkedIn, X/t.co, and
an RTKLIB technical blog. Referrers are context for choosing a channel, not
proof that a channel caused a star or a conversion.

## Seven-day staged sequence

Leave enough time between steps to attribute a change in the local snapshots.
The default spacing is at least 36 hours; do not start the next channel while
the previous one still has unanswered technical questions.

| Day | Channel and purpose | Gate and measurement |
|---:|---|---|
| 0 | Baseline, preflight, and maintainer review | Capture a snapshot immediately before the first post; verify all links and claims. |
| 1 | [Show HN](https://news.ycombinator.com/showhn.html): invite demo/technical feedback | Post only if the current Show HN rules fit; capture at 24 hours and answer substantive questions. |
| 3 | [ROS Discourse](https://discourse.ros.org/t/community-software-release-announcements-guideline/1305): make the ROS2 getting-started path discoverable | Follow the current release guideline and support-route expectations; capture before and after the post. |
| 5 | Existing LinkedIn/X audience: concise release and demo pointer | Use the maintainer's existing audience, disclose affiliation, and avoid repeated cross-post text. |
| 7 | Optional niche technical forum, only after a fresh rule check | Post only when self-promotion and affiliation disclosure are clearly allowed; otherwise skip and record the reason. |

Reddit and `r/cpp` are **not** assumed to allow promotion. They are an
optional rule-gated idea only: read the current community rules, verify that a
maintainer release with self-promotion is permitted, and skip when the rule or
moderator expectation is unclear. No lack of a Reddit post is a launch failure.

## Ready-to-paste drafts

Replace no links with tracking redirects and do not add a star/vote request.
These drafts are starting points for a maintainer to edit after the channel
rule check.

### Show HN

**Show HN title**

`Show HN: libgnss++ v0.2.0 – C++20 GNSS toolkit with an offline demo`

**Body**

```text
I maintain libgnss++, a C++20 GNSS toolkit for SPP/RTK/PPP, RTCM, UBX/SBF,
ROS2 playback, and QZSS L6 correction inspection. v0.2.0 includes a tracked
offline PPP demo so the first artifact can be checked without a receiver or
network service.

Start here:
- Repository: https://github.com/rsasaki0109/gnssplusplus-library
- Release: https://github.com/rsasaki0109/gnssplusplus-library/releases/tag/v0.2.0
- Offline demo: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/self_contained_demo.md
- Use-case routes: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases.md

I am looking for technical feedback on the copy/paste path, artifact schema,
and boundary documentation. The project has explicit validation and benchmark
notes; the demo is a wiring check, not a field-accuracy or deployment claim.
I maintain the repository and will answer issues and reproducibility questions.
This is a request for technical feedback, not for stars or votes.
```

### ROS Discourse

**ROS Discourse title**

`[Release] libgnss++ v0.2.0: offline demo and ROS2 receiver/bag replay path`

**Body**

```text
I maintain libgnss++ and am sharing v0.2.0 for ROS2 and GNSS developers.
The first step is an offline demo; the ROS2 route then runs ros2-doctor,
ros2-bag-doctor, and the existing receiver/bag processor path.

Release: https://github.com/rsasaki0109/gnssplusplus-library/releases/tag/v0.2.0
Demo: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/self_contained_demo.md
ROS2 guide: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/ros2.md
Support: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/SUPPORT.md

The guide documents dependency checks, skip behavior when ROS2 or a bag is
missing, expected JSON/.pos artifacts, and the boundary around production
real-time safety. Please use the issue tracker/support route for bugs or
reproduction details. I am the maintainer; this is a release announcement,
not a request for votes or stars.
```

### LinkedIn/X

```text
libgnss++ v0.2.0 is out: a C++20 GNSS toolkit with a tracked offline PPP demo,
RTKLIB migration, ROS2 bag replay, and QZSS L6/CLAS/MADOCA getting-started
routes. I maintain the project and welcome technical feedback.

Release: https://github.com/rsasaki0109/gnssplusplus-library/releases/tag/v0.2.0
Demo: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/self_contained_demo.md
Routes: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases.md
```

### Optional technical forum

Use this only after the current forum rule gate passes. Keep the affiliation
disclosure in the post rather than hiding it in a profile:

```text
Maintainer disclosure: I maintain libgnss++ and am sharing its v0.2.0 release
under this forum's current software-promotion rules. The useful first artifact
is the offline demo; the technical routes cover RTKLIB migration, ROS2 bag
replay, and QZSS L6/CLAS/MADOCA inspection.

Release: https://github.com/rsasaki0109/gnssplusplus-library/releases/tag/v0.2.0
Technical routes: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases.md
RTKLIB route: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/rtklib_migration.md
ROS2 route: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/ros2.md
QZSS route: https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/docs/use_cases/qzss_l6.md

I am asking for reproducibility or interface feedback, not votes, likes, or
stars. I will follow the forum's support and disclosure rules.
```

## Snapshot and comparison commands

The collector uses only `GH_TOKEN` or `GITHUB_TOKEN`, never prints the token,
and requires owner-level visibility for traffic endpoints. Supply the secret
through the calling environment or secret manager; do not put it in a command
line, a tracked file, or a launch note. A 401/403 is a gate failure to resolve,
not a reason to try another person's token. If owner traffic is unavailable,
retain only the public page/release evidence and mark the traffic measurement
skipped.

All outputs below stay under the ignored `output/growth/` directory. Never
commit the JSON or Markdown files: they can contain owner-only traffic values.

Capture a baseline immediately before the first post:

```bash
mkdir -p output/growth
python3 scripts/metrics/capture_growth_snapshot.py \
  --repo rsasaki0109/gnssplusplus-library \
  --output output/growth/baseline.json \
  --markdown output/growth/baseline.md
```

Capture each later checkpoint with a distinct filename and compare it to the
baseline. The output and comparison input must be different files:

```bash
python3 scripts/metrics/capture_growth_snapshot.py \
  --repo rsasaki0109/gnssplusplus-library \
  --output output/growth/day-3.json \
  --markdown output/growth/day-3.md \
  --compare-to output/growth/baseline.json
```

For deterministic local tests, pass an explicit `--captured-at
2026-08-12T00:00:00Z`. For a live launch, omit it so the script records the
UTC capture time. Snapshot at baseline, 24 hours, 72 hours, and day 7; add a
before/after snapshot when a channel is delayed or skipped.

The comparison reports the star delta and gap to 200, plus approximate
rolling-window views/clones and release-asset download deltas. A clone is not
a user, a view is not a person, an asset download is not an installation, and
a star is not a channel conversion. Keep those distinctions in any report.

## Response and measurement checklist

For every channel:

- Answer technical questions from the repository's support route and link to
  the exact guide or validation note that resolves them.
- Correct a misleading claim publicly and record the correction in the local
  launch notes.
- Disclose maintainer affiliation when a reply could otherwise look like an
  independent endorsement.
- Do not ask friends or collaborators to vote, like, star, or repeat the post.
- At 24h, 72h, and day 7, save the snapshot path, channel timestamp, post URL,
  substantive replies, and any rule-gate skip reason.

Use these decision thresholds as operating rules, not as causal conclusions:

- **24h:** preserve the baseline and one checkpoint. If there are no
  substantive questions, do not compensate with another channel immediately.
- **72h:** if the star delta is at least 5 or there are at least two useful
  technical replies/referrals, continue the staged sequence and prioritize
  answering them. If the delta is below 2 and there is no qualified referral,
  hold the next channel and inspect message/route fit.
- **Day 7:** at least 17 additional stars reaches the stated 200 target; stop
  amplification and archive the evidence. A delta of 5–16 permits one
  documentation-focused follow-up only when there is substantive engagement.
  A delta of 0–4 means stop the sequence, improve the demo/use-case route, and
  do not start a larger cross-post campaign.

These thresholds do not attribute stars to a specific post. GitHub's traffic
window rolls for 14 days, so adjacent snapshots overlap; compare cadence and
window notes, not a fabricated per-channel conversion rate. The maintainer's
decision record should contain the public baseline, local snapshot filenames,
channel timestamps, rule checks, and next action while excluding raw tokens and
owner-only files from Git.
