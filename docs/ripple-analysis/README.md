# Calibration Artifacts

Task 3 owns host-only authenticated replay admission. A replay may print
`calibration_status=calibrated` only after its KV SHA-256 covers the schema,
status, source-data digest, manifest provenance digest, and every physical
parameter, followed by full runtime parameter and per-scan input admission.

Task 2 physical validation is not calibration evidence and does not authorize
calibrated replay or production ripple-table export.

Production-table inputs must be finite, strictly increasing absolute RPM breakpoints
in `0..2000`, phases in `[-pi, pi]` radians, and signed RPM amplitudes no larger than
30% of the corresponding RPM breakpoint. The exporter treats the authenticated KV as
an artifact-consistency anchor; this detects mismatched generated artifacts, not a
malicious writer with repository access.

The consistency graph is `KV -> params -> manifest provenance -> summary -> validation`.
Each digest is calculated over canonical sorted-key JSON with its own digest field
excluded. The manifest provenance digest additionally excludes derived
`calibration_id`, because that ID itself binds the manifest digest; all other manifest
admission fields, including `missing_provenance`, remain covered.
