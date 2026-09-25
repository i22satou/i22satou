---
name: find-skills
description: Helps users discover and install agent skills when they ask questions like "how do I do X", "find a skill for X", "is there a skill that can...", or express interest in extending capabilities. This skill should be used when the user is looking for functionality that might exist as an installable skill.
---

# Find Skills

This skill helps you discover and install skills from the open agent skills ecosystem.

## The Skills CLI

The Skills CLI (`npx skills`) is the package manager for the open agent skills ecosystem. Skills are modular packages that extend agent capabilities with specialized knowledge, workflows, and tools.

- `npx skills find [query] [--owner <owner>]` - Search for skills by keyword, optionally scoped to a GitHub owner
- `npx skills add <package>` - Install a skill from GitHub or other sources
- `npx skills update` - Update all installed skills
- `npx skills init <name>` - Create a new skill

Browse and compare skills at https://skills.sh/ - its leaderboard ranks skills by total installs.

## Finding a skill

The goal is to recommend a skill the user can trust, or to say plainly that none fits. Work out the domain and the specific task from the user's request, check the skills.sh leaderboard for well-known skills in that domain, and search with specific keywords (`npx skills find react testing` rather than `testing`), trying alternative terms if the first search comes up empty.

Search results alone are not a recommendation. Before recommending a skill, check its install count, its source (official publishers such as `anthropics`, `vercel-labs`, or `microsoft` are more trustworthy than unknown authors), and the stars on its source repository. Treat skills with few installs or from little-known repositories with caution, and say so when you recommend one.

When you present a skill, give its name and what it does, its install count and source, the install command, and its skills.sh link.

## Installing

Install a skill only after the user has agreed to that specific skill, because installation adds third-party instructions that will run in later sessions. Ask whether to install it for this project or globally, then run `npx skills add <owner/repo@skill>` (add `-g` for a global install). Let the CLI show its confirmation prompt rather than skipping it with `-y`.

## When no skill fits

Tell the user no suitable skill was found, offer to help with the task directly, and mention that they can create their own with `npx skills init` if it is something they do often.
