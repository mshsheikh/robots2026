#!/usr/bin/env bash
FEATURE="001-docusaurus-robotics"
HASH=$(git rev-parse --short HEAD)
MSG=$(git log -1 --pretty=%B)
FILES=$(git show --name-only --pretty="" HEAD | sed '/^$/d')
TS=$(date +"%Y%m%dT%H%M%S")
OUT="history/prompts/$FEATURE/${TS}.${HASH}.phr.md"
mkdir -p "history/prompts/$FEATURE"
cat > "$OUT" <<PHR
---
id: $TS
commit: $HASH
summary: $MSG
---

## Files changed
\`\`\`
$FILES
\`\`\`
PHR
git add "$OUT" || true
echo "PHR created: $OUT"
