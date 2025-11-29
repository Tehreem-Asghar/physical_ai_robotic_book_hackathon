<!--
Sync Impact Report:
- Version change: 0.0.1 → 0.0.2
- Modified principles: None
- Added sections: None
- Removed sections: None
- Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
- Follow-up TODOs: TODO(RATIFICATION_DATE): Clarify original adoption date if different from creation date.
-->
# Project Constitution: Physical AI & Humanoid Robotics Textbook

## 1. Governance

**CONSTITUTION_VERSION**: 0.0.2
**RATIFICATION_DATE**: 2025-11-29
**LAST_AMENDED_DATE**: 2025-11-29

### 1.1 Amendment Procedure

This constitution can be amended by consensus of the project leads. All proposed amendments MUST be documented, reviewed, and approved before integration. Minor clarifications or typo fixes will result in a PATCH version bump. New principles or significant expansions will trigger a MINOR version bump. Backward incompatible changes (e.g., removal of a core principle) will require a MAJOR version bump.

### 1.2 Versioning Policy

The constitution follows Semantic Versioning (MAJOR.MINOR.PATCH).
- **MAJOR** version increments for backward incompatible changes, removal, or redefinition of core governance or principles.
- **MINOR** version increments for new principles, sections, or materially expanded guidance.
- **PATCH** version increments for clarifications, wording, typo fixes, or non-semantic refinements.

### 1.3 Compliance Review

Adherence to this constitution will be reviewed periodically by project leads to ensure ongoing alignment with project goals and standards. Deviations MUST be addressed promptly.

## 2. Core Principles

### 2.1 Accuracy

All technical content MUST be verified from credible AI, robotics, and academic sources. Content MUST be factually correct and up-to-date with current industry and research standards.

### 2.2 Clarity

Text MUST be understandable by senior Computer Science and Robotics students. The Flesch-Kincaid grade level MUST be maintained between 10-12 to ensure appropriate readability for the target audience. Technical jargon MUST be explained clearly.

### 2.3 Reproducibility

Any code, simulation, or experiment described in the book SHOULD be reproducible from the provided instructions. All necessary dependencies and setup steps MUST be clearly documented to enable users to replicate results.

### 2.4 Rigor

Content MUST include industry standards, best practices, and relevant theoretical foundations in AI and robotics. Concepts MUST be presented with appropriate depth and technical detail.

### 2.5 Safety

All instructions for hardware use (e.g., robots, Jetson kits) MUST explicitly highlight safe handling procedures, potential hazards, and necessary precautions to protect users and equipment.

## 3. Key Standards

### 3.1 Citations

All claims and external information MUST cite credible sources using APA format. Direct quotes or paraphrased content MUST be properly attributed.

### 3.2 Code & Simulation

The book MUST include tested, verified code examples and instructions for common robotics and AI platforms such as ROS2, Gazebo, Isaac, and Visual Learning Agents (VLA). Code MUST be functional and demonstrative of the concepts discussed.

### 3.3 Plagiarism

There is a zero-tolerance policy for plagiarism. No copied text without proper citation is permitted. All content MUST be original or appropriately attributed.

### 3.4 Hardware Instructions

All hardware-related instructions MUST include comprehensive safety guidelines and detailed setup notes to ensure correct and secure implementation.

## 4. Constraints

### 4.1 Format

The book content MUST be written in Markdown, fully compatible with Docusaurus for static site generation. This includes adherence to Markdown syntax and Docusaurus-specific features.

### 4.2 Accessibility

Content MUST be accessible, including clear diagrams, well-formatted tables, and step-by-step instructions to enhance understanding for diverse learners.

### 4.3 Language

The primary language for the textbook is English. Provisions for an optional Urdu translation may be considered for future editions.

## 5. Success Criteria

### 5.1 Verifiable Content

Every module MUST have verifiable sources for technical claims and working, tested code examples.

### 5.2 Safety Documentation

Comprehensive safety instructions for all hardware components and experimental procedures MUST be included.

### 5.3 Target Audience Readability

The content MUST be readable and comprehensible for the target audience of senior Computer Science and Robotics students, adhering to the specified Flesch-Kincaid grade level.

### 5.4 Chapter Specification Readiness

The constitution aims to establish a foundational framework that enables the smooth generation of detailed specifications for each chapter of the textbook.
