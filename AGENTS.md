# Repository Instructions

## Commit message format

Use Conventional Commit subjects:

```text
<type>(<scope>): <short imperative description>
```

Omit `(scope)` only when the change clearly applies to the whole repository:

```text
<type>: <short imperative description>
```

Allowed types:

- `feat`: add or extend functionality
- `fix`: correct faulty behavior
- `docs`: change documentation only
- `test`: add or update tests
- `refactor`: restructure code without changing behavior
- `build`: change firmware build or packaging
- `chore`: repository maintenance

Preferred scopes:

- `main`: real-airframe firmware
- `hitl`: hardware-in-the-loop support
- `sitl`: software-in-the-loop support
- `l1`: shared L1 controller code

Rules:

- Use lowercase for the type and scope.
- Write the description in English, starting with an imperative verb.
- Do not end the subject with a period.
- Keep the subject concise, preferably no more than 72 characters.
- Inspect recent commit subjects before creating a commit and follow their style.
- Do not add automated attribution or co-author trailers.

Examples:

```text
feat(main): add configurable circle trajectory speed
fix(hitl): preserve yaw during motor degradation
docs: document firmware default parameter workflow
test(l1): cover runtime trajectory parameter updates
```
