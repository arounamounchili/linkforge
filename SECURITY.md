# Security Policy

## Supported Versions

We provide security updates and patches for the following versions:

| Version | Supported |
| :--- | :--- |
| **1.5.x (Latest: 1.5.2)** | :white_check_mark: Full Support |
| **1.4.x** | :white_check_mark: Critical Fixes |
| **1.3.x** | :white_check_mark: Critical Fixes |
| **< 1.3.0** | :x: Unsupported |

---

## Security Scope & Threat Model

LinkForge processes external, third-party robot descriptions and meshes. Our security architecture enforces:

* **Resource Sandboxing**: Resource paths (`package://` and relative file paths) are strictly jailed to declared package roots, preventing path traversal attacks.
* **Safe Expression Evaluation**: XACRO mathematical expressions are evaluated within a restricted scope without access to arbitrary Python built-ins or system commands.
* **Hardened XML Ingestion**: Defense against XML entity expansion (e.g. Billion Laughs attacks) and quadratic blowout.

---

## Reporting a Vulnerability

If you discover a security vulnerability within LinkForge, please **do NOT report it via public GitHub issues or discussions**.

Instead, please report it privately through one of the following channels:

1. **GitHub Private Vulnerability Advisory (Preferred)**:
   Submit a confidential advisory directly via [GitHub Security Advisories](https://github.com/arounamounchili/linkforge/security/advisories/new).
2. **Direct Email**:
   Email the lead maintainer directly at [patouossa.mounchili@gmail.com](mailto:patouossa.mounchili@gmail.com) with the subject line `[SECURITY] LinkForge Vulnerability Report`.

### What to Include
* A description of the vulnerability and its potential impact.
* Minimal reproduction steps or proof-of-concept files (e.g. a sample URDF/XACRO).
* The affected component (`linkforge-core` or `linkforge-blender`) and version.

### Response Timeline
* **Initial Acknowledgment**: Within 48 hours.
* **Triage & Severity Assessment**: Within 5 business days.
* **Coordinated Disclosure**: Fixes will be prepared in a private fork and published alongside an official release and CVE attribution (unless you prefer anonymity).
