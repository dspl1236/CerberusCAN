# Example scan — 2013 Audi A6 C7 (3.0T)

A real `SCAN` + identify run from CerberusCAN on a 2013 Audi A6 C7, to show what the
tool finds on a VAG car.

**Every module is reached on Head 1** (the diagnostic bus, OBD 6/14): the J533 gateway
routes *all* UDS — including the gateway-routed comfort modules (HVAC, seats) over VW TP 2.0
— onto the OBD diagnostic CAN, so one head maps the whole car. Head 2 taps the *same* 6/14
bus listen-only, so it logs exactly this traffic as you drive it. (The gateway firewalls the
OBD port to the diagnostic CAN, so there's no separate comfort bus to tap there — see
[HARDWARE.md](HARDWARE.md).)

Discovered with `SCAN:1`, then identified by reading `F187` (part number) and `F19E`
(ASAM ODX name) from each module.

| Head | Req/Resp | Part # | ODX name | Module |
|:----:|----------|--------|----------|--------|
| 1 | `710/77A` | 4G0907468AC | EV_GatewPkoUDS | Gateway (J533) |
| 1 | `712/77C` | 4G0909144L | EV_RCEPS | Electric power steering (J500) |
| 1 | `713/77D` | 4G0907379H | EV_ESPPremiAU57X | ABS / ESP (J104) |
| 1 | `714/77E` | 4G8920983E | EV_RBD4K | Instrument cluster (J285) |
| 1 | `716/77F` | — | — | responds, no part/name (sub-controller?) |
| 1 | `70A/774` | 4H0919475AA | EV_EPHVA18 | Park assist (J791) |
| 1 | `721/78B` | 4G0907637K | EV_SARA2 | Sensor module (SARA2) |
| 1 | `746/7B0` | 4G0820043L | EV_AirCondiComfoUDS | HVAC / Climatronic (J255) |
| 1 | `752/7BC` | 4H0907801F | EV_ParkiBrake | Electronic parking brake (J540) |
| 1 | `754/7BE` | 4H0907357B | EV_HeadlRegulBasic | Headlight range control (J431) |
| 1 | `75E/7C8` | 4H0980945B | EV_RGS_L | Side sensor, left |
| 1 | `75F/7C9` | 4H0980946B | EV_RGS_R | Side sensor, right |
| 1 | `7E0/7E8` | 4G0907551D | EV_ECM30TFS… | Engine — Simos8.5 3.0T (J623) |
| 1 | `7E1/7E9` | 4G1927158A | EV_TCMAL551211 | Transmission — ZF8HP / AL551 (J217) |

**14 modules, 13 named.**

> A raw `SCAN` reports ~30 hits because IDs `0x700–0x70F` are VAG **functional**
> broadcast addresses (each elicits a real module's reply). The physical modules are
> the ones where `response = request + 0x6A` (body) or `+ 0x08` (powertrain), which is
> the deduped list above.

## Reproduce

```
SCAN:1                  # sweep 0x700-0x7EF, list every responder
710:77A:22F187          # read a module's part number
710:77A:22F19E          # read a module's ASAM ODX name
```
