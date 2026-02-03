# Reliability Diagnosis Report

## Stability Summary

| Cell | Status | Pass Rate | Infra Fails | Semantic Fails |
|---|---|---|---|---|
| humble/prod/L05 | PASS | 5/5 | 0 | 0 |
| humble/prod/P04 | PASS | 5/5 | 0 | 0 |
| humble/prod/S11 | PASS | 5/5 | 0 | 0 |
| humble/rclpy/L05 | FAIL | 0/5 | 0 | 5 |
| humble/rclpy/P04 | PASS | 5/5 | 0 | 0 |
| humble/rclpy/S11 | FAIL | 0/5 | 0 | 5 |
| jazzy/prod/L05 | PASS | 5/5 | 0 | 0 |
| jazzy/prod/P04 | PASS | 5/5 | 0 | 0 |
| jazzy/prod/S11 | PASS | 5/5 | 0 | 0 |
| jazzy/rclpy/L05 | FAIL | 0/5 | 0 | 5 |
| jazzy/rclpy/P04 | PASS | 5/5 | 0 | 0 |
| jazzy/rclpy/S11 | FAIL | 0/5 | 0 | 5 |

## Failure Analysis

### humble/rclpy/L05
- **run_1** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/L05/run_1/trace.jsonl`
  - Report: `evidence/humble/rclpy/L05/run_1/report.json`
  - Log: `evidence/humble/rclpy/L05/run_1/stdout.log`
- **run_2** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/L05/run_2/trace.jsonl`
  - Report: `evidence/humble/rclpy/L05/run_2/report.json`
  - Log: `evidence/humble/rclpy/L05/run_2/stdout.log`
- **run_3** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/L05/run_3/trace.jsonl`
  - Report: `evidence/humble/rclpy/L05/run_3/report.json`
  - Log: `evidence/humble/rclpy/L05/run_3/stdout.log`
- **run_4** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/L05/run_4/trace.jsonl`
  - Report: `evidence/humble/rclpy/L05/run_4/report.json`
  - Log: `evidence/humble/rclpy/L05/run_4/stdout.log`
- **run_5** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/L05/run_5/trace.jsonl`
  - Report: `evidence/humble/rclpy/L05/run_5/report.json`
  - Log: `evidence/humble/rclpy/L05/run_5/stdout.log`

### humble/rclpy/S11
- **run_1** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/S11/run_1/trace.jsonl`
  - Report: `evidence/humble/rclpy/S11/run_1/report.json`
  - Log: `evidence/humble/rclpy/S11/run_1/stdout.log`
- **run_2** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/S11/run_2/trace.jsonl`
  - Report: `evidence/humble/rclpy/S11/run_2/report.json`
  - Log: `evidence/humble/rclpy/S11/run_2/stdout.log`
- **run_3** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/S11/run_3/trace.jsonl`
  - Report: `evidence/humble/rclpy/S11/run_3/report.json`
  - Log: `evidence/humble/rclpy/S11/run_3/stdout.log`
- **run_4** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/S11/run_4/trace.jsonl`
  - Report: `evidence/humble/rclpy/S11/run_4/report.json`
  - Log: `evidence/humble/rclpy/S11/run_4/stdout.log`
- **run_5** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/humble/rclpy/S11/run_5/trace.jsonl`
  - Report: `evidence/humble/rclpy/S11/run_5/report.json`
  - Log: `evidence/humble/rclpy/S11/run_5/stdout.log`

### jazzy/rclpy/L05
- **run_1** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/L05/run_1/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/L05/run_1/report.json`
  - Log: `evidence/jazzy/rclpy/L05/run_1/stdout.log`
- **run_2** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/L05/run_2/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/L05/run_2/report.json`
  - Log: `evidence/jazzy/rclpy/L05/run_2/stdout.log`
- **run_3** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/L05/run_3/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/L05/run_3/report.json`
  - Log: `evidence/jazzy/rclpy/L05/run_3/stdout.log`
- **run_4** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/L05/run_4/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/L05/run_4/report.json`
  - Log: `evidence/jazzy/rclpy/L05/run_4/stdout.log`
- **run_5** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/L05/run_5/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/L05/run_5/report.json`
  - Log: `evidence/jazzy/rclpy/L05/run_5/stdout.log`

### jazzy/rclpy/S11
- **run_1** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/S11/run_1/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/S11/run_1/report.json`
  - Log: `evidence/jazzy/rclpy/S11/run_1/stdout.log`
- **run_2** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/S11/run_2/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/S11/run_2/report.json`
  - Log: `evidence/jazzy/rclpy/S11/run_2/stdout.log`
- **run_3** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/S11/run_3/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/S11/run_3/report.json`
  - Log: `evidence/jazzy/rclpy/S11/run_3/stdout.log`
- **run_4** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/S11/run_4/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/S11/run_4/report.json`
  - Log: `evidence/jazzy/rclpy/S11/run_4/stdout.log`
- **run_5** (SEMANTIC_FAIL): Missing: op_end
  - Trace: `evidence/jazzy/rclpy/S11/run_5/trace.jsonl`
  - Report: `evidence/jazzy/rclpy/S11/run_5/report.json`
  - Log: `evidence/jazzy/rclpy/S11/run_5/stdout.log`

