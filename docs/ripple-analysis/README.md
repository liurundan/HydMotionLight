# Calibration Artifacts

Task 3 owns host-only authenticated replay admission. A replay may print
`calibration_status=calibrated` only after its KV SHA-256 covers the schema,
status, source-data digest, manifest provenance digest, and every physical
parameter, followed by full runtime parameter and per-scan input admission.

Task 2 physical validation is not calibration evidence and does not authorize
calibrated replay or production ripple-table export.
