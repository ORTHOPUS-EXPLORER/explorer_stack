@AGENTS.md

# Project coding guidelines

These add to the guide above.

## C++

- **Follow this project's style, not an outside style guide's**: take naming and formatting from `.clang-format`, `.clang-tidy` and the surrounding code. Don't bring in conventions this project doesn't use, even when a tool or guide it borrows from has them (e.g. `.clang-format` is based on Google style, but Google naming isn't used here):

  ```cpp
  // Avoid: Google-style kCamelCase constant
  constexpr double kMaxSpeed = 1.0;

  // Prefer: UPPER_SNAKE_CASE constant, as in the rest of the project
  constexpr double MAX_SPEED = 1.0;
  ```

- **No compound bitwise assignment operators** (`|=`, `&=`, `^=`, `<<=`, `>>=`). Write the intent explicitly instead, with a plain `if` or a full expression:

  ```cpp
  // Avoid
  write_failed |= !interface.set_value(command);

  // Prefer
  if (!interface.set_value(command))
  {
    write_failed = true;
  }
  ```

  Arithmetic compound assignments (`+=`, `-=`, `*=`, `/=`) are fine.

- **Mark functions `[[nodiscard]]` when ignoring the result is a bug**: getters, pure computations, and functions returning a status or error (e.g. `bool` success). Don't add it to functions called mainly for their side effects, where the result is optional information.

## Tests

- **One behavior per test**: each test checks one specific behavior, so a failure points to a single cause. Several assertions are fine when they all describe that same behavior.
- **A short comment before each test**: one simple, human readable sentence saying what behavior is checked (e.g. `// compute : a command too far from the measured position is clamped to the threshold`), starting with what's being tested like 'constructor :' or '$METHOD_NAME :'.
- **Test user-facing behavior, not implementation details**: assert on what callers observe (outputs, published messages, interfaces, parameter effects), not on internal state or private helpers, so refactors don't break tests.
- **Explain computed expected values**: when an expected value comes from a calculation, say in the `// Assert` comment how it is obtained (e.g. `// Assert: 0.1 + 0.2 threshold`). When it helps, also give the value a wrong implementation would produce (e.g. `// 0.4 if the shift were lost`).
- **Order tests like the code they test**: define a method's tests in the order their cases appear in it (early returns and guards first, then the main path), so the tests can be read side by side with the method. Not always possible; follow it as closely as the code allows.
