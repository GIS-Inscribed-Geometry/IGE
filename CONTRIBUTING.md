# Contributing to IGE

Thank you for your interest in contributing to the Inscribed Geometry Engine!

## Getting Started

1. Fork the repository
2. Clone your fork
3. Create a feature branch (`git checkout -b feature/my-feature`)
4. Make your changes
5. Run tests (`cargo test --workspace --all-features`)
6. Run clippy (`cargo clippy --workspace --all-features -- -D warnings`)
7. Run formatter (`cargo fmt --all --check`)
8. Commit and push
9. Open a pull request

## Development

This project uses nightly Rust as specified in `rust-toolchain.toml`.

### Python bindings

If working on the Python bindings in `crates/ige-py`, install maturin and test with:

```bash
maturin develop --manifest-path crates/ige-py/Cargo.toml
pytest
```

## Pull Request Process

1. Ensure all CI checks pass
2. Update documentation if needed
3. Add tests for new functionality
4. Update the changelog if one exists

## Code of Conduct

Please note that this project adheres to the [Code of Conduct](CODE_OF_CONDUCT.md).
By participating, you are expected to uphold this code.
