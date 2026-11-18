# LX200 command support (GM K150 firmware)

Reference: Meade *Telescope Serial Command Protocol* (2010-10), plus 10Micron extensions.
Replies end with `#` unless noted. On the strict port (Serial1 / WiFi bridge) the `\n`
after replies is dropped and single-character quick commands are ignored.

Status: **ok** = implemented as specified · **dummy** = fixed answer · **silent** = accepted, no
reply (as LX200 defines for that command) · **--** = not implemented (no reply).

Used by: **S** = SkySafari (Meade LX200 Classic), **St** = Stellarium (desktop / Mobile Plus),
**I** = INDI/KStars `lx200generic` (from their documented behaviour; verify with your app).

## Required for connect, position display, GOTO, sync

| Command | Meaning | Reply | Status | Used by |
|---|---|---|---|---|
| `<ACK>` (0x06, no `:`) | alignment mode query / link probe; **switches that port to strict LX200** | `P` (no `#`) | ok | S St I |
| `:GR#` | telescope RA | `HH:MM:SS#` | ok | S St I |
| `:GD#` | telescope Dec | `sDD*MM:SS#` | ok | S St I |
| `:Sr HH:MM:SS#` (also `HH:MM.T`) | target RA | `1` / `0` | ok | S St I |
| `:Sd sDD*MM:SS#` (also `'`, `sDD*MM`) | target Dec | `1` / `0` | ok | S St I |
| `:MS#` | GOTO target | `0` ok, `1<msg>#` below horizon, `2<msg>#` limits / not synced | ok | S St I |
| `:CM#` | sync to target | `<text>#` | ok | S St I |
| `:Q#` | stop all slews (resume tracking) | -- | ok | S St I |
| `:D#` | distance bars | `-...#`, `#` when done | ok | I |
| `:Mn# :Ms# :Me# :Mw#` | move N/S/E/W at slew rate; after `:RG#` via the guide engine (INDI guiding without pulse cmd) | -- | ok | S St I |
| `:Qn# :Qs# :Qe# :Qw#` | halt that direction | -- | ok | S St I |
| `:RG# :RC# :RM# :RS#` | slew rate guide/center/find/max | -- | ok | S St I |
| `:U#` | precision toggle | -- | silent (always high) | S I |
| `:hP#` | park: pole, counterweight down, tracking off (needs a sync) | -- | ok | I |

## Site, time, date (sent on connect if the app is set to)

| Command | Meaning | Reply | Status | Used by |
|---|---|---|---|---|
| `:Sg sDDD*MM#` | longitude, **west positive** | `1` / `0` | ok | S I |
| `:St sDD*MM#` | latitude | `1` / `0` | ok | S I |
| `:SG sHH.H#` | hours to add to local time for UTC | `1` | ok | S I |
| `:SL HH:MM:SS#` | local time | `1` / `0` | ok | S I |
| `:SC MM/DD/YY#` | local date | `1Updating Planetary Data#` + 30 spaces + `#` | ok | S I |
| `:Gg#` `:Gt#` | longitude / latitude | `sDDD*MM#` / `sDD*MM#` | ok | I |
| `:GG#` | UTC offset | `sHH#` | ok | I |
| `:GL#` `:GC#` | local time / date | `HH:MM:SS#` / `MM/DD/YY#` | ok | I |
| `:GS#` | local sidereal time | `HH:MM:SS#` | ok | I |
| `:Gc#` | clock format | `24#` | ok | I |
| `:GM#` | site name | `GM-K150#` | ok | I |

## Informational

| Command | Meaning | Reply | Status |
|---|---|---|---|
| `:GVP# :GVN# :GVD# :GVT#` | product / version / date / time | `...#` | ok |
| `:GW#` | status: mount, tracking, alignment | `G` + `T`/`N` + `1`/`0` + `#` | ok |
| `:GA#` `:GZ#` | altitude / azimuth | `sDD*MM:SS#` / `DDD*MM:SS#` | ok |
| `:Gr#` `:Gd#` | target RA / Dec | as `:GR#` / `:GD#` | ok |
| `:GT#` | tracking rate (60 Hz = sidereal) | `TT.T#` | ok |
| `:Gm#` | distance to meridian | `00:00:00#` | dummy |
| `:Gf# :Gl# :GF# :GE# :Ge#` | browse limits, field, selenographic | fixed | dummy |
| `:GRTMP# :GRPRS#` | refraction temp / pressure (10Micron) | `5.0#` / `1011.0#` | dummy |
| `:Ginfo#` | 10Micron status line | `ra,dec,E,az,alt,jd,gstat,dist#` | ok |
| `:modelcnt# :getalst#` | 10Micron model info | `0#` | dummy |

## Tracking

| Command | Meaning | Reply | Status |
|---|---|---|---|
| `:TQ# :TS# :TL#` | sidereal / solar / lunar | -- | ok |
| `:T+# :T-#` | rate +- 0.1 (60 Hz scale) | -- | ok (see note on rate resolution) |
| `:TM#` | custom rate | -- | ok |
| `:SwN#` | max slew rate | `1` | accepted, GOTO_SPEED fixed |

## Guiding

| Command | Meaning | Reply | Status |
|---|---|---|---|
| `:Mgn# :Mgs# :Mge# :Mgw#` + `NNNN` | pulse guide NNNN ms (max 10000) | -- | ok |
| `:RgSS.S#` | guide rate in arcsec/s (max sidereal 15.04) | -- | ok (default 0.5x = 7.5"/s) |
| `:Rg#` | K150: report guide rate | `SS.S#` | ok |

RA guiding changes the tracking rate to (1 +- guide rate) x sidereal -- the RA motor never
reverses. Dec runs at +- guide rate. The ST-4 input uses the same engine and rate.

## Accepted silently / not implemented

`:hR#`, `:H#`, `:I#`, `:L..#` (library), `:W1-4#` (site select), `:g+# :g-#` (GPS),
`:V..#` (PEC), `:f..#` (fan), `:B..#` (reticle), `:AL# :AP# :AA#` -- no reply.

Not implemented (no reply, apps time out on them): `:Sa# :Sz# :MA#` (alt/az targets), `:GM#`..`:GP#` site names, `:ST#` tracking frequency,
`:Sh# :So#` horizon/limits, `:P#`.

## K150 extensions (custom) and overlaps with standard commands

| Command | K150 meaning | Standard LX200 meaning |
|---|---|---|
| `:GK#` / `:SK h;d#` | raw axis positions (microsteps) | -- |
| `:QM h;d#` | absolute raw move | -- |
| `:QV# :Qv# :QF# :Qf#` | select slew rate + motor setup | -- |
| `:QR# :Qr# :QD# :Qd#` | start H+/H-/D+/D- slew (main.py) | -- |
| `:Q0# :QX# :Q*#` | stop / emergency stop / stop | -- |
| `:T># :T<# :T0#` `[HD]` | tracking direction / off per axis | -- |
| `:TH# :TD# :TC..# :TCR# :G~#` | K150 tracking control | -- |
| `:MT0# :MT1#` | closed-loop HA tracking off/on | -- |
| `:R*R# :R*S# :R*C# :RR#` | slew rate table | -- |
| `:F+# :F-# :FQ#` | focuser in / out / halt, no reply | same (INDI focuser) |
| `:FS#` | focuser halt | slow speed -- harmless (stops) |
| `:FN# :FR# :FP# :F0#` | focuser steps / rate / position / zero | -- |
| `:%..#` `:@#` `:?#` | keypad, tests, help | -- |
| **`:GN#`** | raw positions in hex (main.py) | site 2 name -- conflict, apps rarely ask |
| **`:SM..#`** | motor speed/power setup | set site 1 name -- conflict, apps rarely send |
| **`:TR#`** | report tracking rates | (Autostar) return to default rate -- conflict, rare |

## Tested with INDI `indi_lx200generic` (1.9.9, device "Standard LX200")

Captured on a virtual serial port against a stand-in for these replies:
connect `<ACK> :GR# <ACK> :Gc# :GM# :GT# :Gt# :Gg# :GG# :GL# :GC#`, then `:GR# :GD#` polling;
GOTO `:Sr :Sd :MS#` then `:D#` until `#`; sync `:Sr :Sd :CM#`; motion `:Mn# / :Qn#`;
slew rate 1x-4x = `:RG# :RC# :RM# :RS#`; abort `:Q#`; park `:hP#` then `:D#`;
site/time from `GEOGRAPHIC_COORD` / `TIME_UTC`: `:Sg072*55# :St+40*57# :SG+5.0# :SL22:30:00# :SC11/17/26#`.
Guiding: right after connect INDI 1.9.9 guides with `:RG# :Mw# ... :Qw#` although "Use Pulse Cmd"
shows On; after toggling it Off/On it sends `:Mgn0300#`. Both now give the same guide-rate correction.
The driver has no TELESCOPE_TRACK_STATE and no GUIDE_RATE property.
