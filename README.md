# ros2-semantic-contracts

ROS 2 standardises interfaces and mechanisms, but it does not provide a single, canonical, testable definition of **semantic correctness** for core behaviours such as lifecycle, actions, and parameters within a defined upstream baseline.

In practice, semantic truth has emerged from implementation and ecosystem convergence — most notably from **rclcpp** and mature stacks such as **Nav2**. Those semantics are widely relied upon, but they are implicit, fragmented, and difficult to validate across languages and versions.

This repository makes that implicit truth **explicit, normative, and testable**.

---

## The Problem

ROS 2 tells you *how to wire things together*.
It does not fully specify *what correct behaviour means* once systems are running, orchestrated, restarted, cancelled, or shut down under real operational pressure.

As a result:

- semantic meaning is inferred from code, not contracts
- behaviour can drift across client libraries without detection
- “works in practice” becomes indistinguishable from “works by accident”
- cross-language parity is assumed rather than proven
- regressions are hard to classify as bugs, changes, or misunderstandings

This is not a tooling failure.
It is a missing specification layer.

---

## The Thesis

**Semantic contracts should be explicit, normative, and testable.**

This repository defines **semantic contracts** for ROS 2 core behaviour:
clear statements of required behaviour, allowed outcomes, and failure responses, independent of language or transport.

These contracts are:

- **Normative** — they define what correct behaviour is
- **Language-agnostic** — they apply equally to C++, Python, Rust, or others
- **Transport-agnostic** — they do not depend on DDS details
- **Observable** — they are judged by externally visible behaviour
- **Enforceable** — they are backed by executable tests and harnesses

The goal is not to redesign ROS.
It is to make existing semantic reality precise.

---

## Baseline Oracle

All contracts in this repository are defined against an explicit upstream baseline.

For the initial contract set, the baseline is:

> **ROS 2 Jazzy + rclcpp + Nav2**

Upstream ROS documentation and REPs are authoritative where they are complete.
Where they are silent or ambiguous, the behaviour of this production stack defines the semantic truth for the purposes of this repository.

That baseline is named, versioned, and treated as the reference oracle against which other implementations are evaluated.

---

## What Is a “Semantic Contract”?

A semantic contract defines:

- what behaviour **must** occur
- what behaviour **must not** occur
- which outcomes are allowed when ordering or timing cannot be fixed
- how invalid states, inputs, or transitions **must be handled**
- what remains explicitly undefined

It does **not** define APIs.
It does **not** prescribe architecture.
It defines observable meaning.

---

## Why `core` Exists

ROS 2 does not lack documentation, quality processes, or testing culture.

It lacks **canonical semantic contracts**.

The `core` specifications in this repository define the *minimum semantic truth* that all compliant implementations must share, independent of language, middleware binding, or adapter strategy.

From those core contracts, the repository supports:

- contract tests
- reference executable semantics
- adapter conformance checks
- divergence detection across implementations
- explicit handling of upstream change vs regression

Without a core semantic layer, parity and correctness cannot be stated — only guessed.

---

## Repository Structure

At a high level, the repository is organised as follows:

```text
docs/
├─ philosophy.md        # how semantic truth is defined and governed
├─ intent.md            # what this repository is and is not
├─ spec/
│  └─ core/             # normative core semantic contracts
├─ provenance/          # citations, baselines, divergence records

reference/              # reference executable semantics (language-specific)
harness/                # oracle runners and comparison tooling
tools/                  # validation, linting, and support scripts
```