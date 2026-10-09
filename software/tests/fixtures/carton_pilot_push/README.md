# Frozen legacy pilot excerpts

These regression fixtures preserve pre-change source excerpts read on 10 October
2026 from the local `gemma-xlerobot/pilot` checkout. Their old pinch-only and retry
wording is intentional test input, not the current operating policy. Do not use
these fixtures as task prompts or rewrite them to score older trials differently.

- `chat_server.py`: literal supervisor prompt, fold/gripper constants, four fold
  helpers, and the four fold/grasp methods under test. Original complete file
  SHA-256: `e42e214342fcdbb5d6c3584c866fe47f3825aafc6c41327541540896deaf3fff`.
- `model_backend.py`: `_nullable`, actions and decision schema only. Original
  complete file SHA-256:
  `e183ddfcec08acd9749883d16391963b9a58c8fb6716a7f65bf424948bbe23e0`.

The fixtures contain no imports, runtime initialization or transport setup.
Tests extract selected AST nodes into fake-only namespaces. The installer also
receives full staged copies for separate source-level verification. The current
policy fragment is `docs/prompts/carton-pilot-push-authorized-v1.txt`.
