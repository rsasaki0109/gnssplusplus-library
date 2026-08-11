#!/usr/bin/env python3
"""Unit and static-contract tests for the v0.2.0 release surface."""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT_DIR / "scripts" / "release" / "validate_version.py"
RELEASE_WORKFLOW_PATH = ROOT_DIR / ".github" / "workflows" / "release.yml"
DOCKER_WORKFLOW_PATH = ROOT_DIR / ".github" / "workflows" / "docker.yml"
PUBLIC_IMAGE = "ghcr.io/rsasaki0109/gnssplusplus-library:v0.2.0"


def load_validator_module():
    spec = importlib.util.spec_from_file_location("release_validate_version", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load validator module from {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


validator = load_validator_module()


class ReleaseVersionValidatorTest(unittest.TestCase):
    def test_valid_v020_matches_cmake(self) -> None:
        self.assertEqual(
            validator.validate_versions("v0.2.0", ROOT_DIR / "CMakeLists.txt"),
            "0.2.0",
        )

    def test_invalid_and_prerelease_tags_are_rejected(self) -> None:
        for tag in ("0.2.0", "v0.2", "v0.2.0-rc1", "v01.2.3"):
            with self.subTest(tag=tag):
                with self.assertRaisesRegex(ValueError, "exact vMAJOR.MINOR.PATCH"):
                    validator.version_from_tag(tag)

    def test_mismatched_tag_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_release_version_") as temp_dir:
            cmake_file = Path(temp_dir) / "CMakeLists.txt"
            cmake_file.write_text(
                "cmake_minimum_required(VERSION 3.14)\n"
                "project(gnss_lib VERSION 0.2.1 LANGUAGES C CXX)\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "declares 0.2.1"):
                validator.validate_versions("v0.2.0", cmake_file)


class ReleaseWorkflowContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.release_workflow = RELEASE_WORKFLOW_PATH.read_text(encoding="utf-8")
        cls.docker_workflow = DOCKER_WORKFLOW_PATH.read_text(encoding="utf-8")

    def test_release_is_tag_only_and_checks_out_the_triggering_tag(self) -> None:
        self.assertIn('on:\n  push:\n    tags:\n      - "v*.*.*"', self.release_workflow)
        self.assertNotIn("workflow_dispatch", self.release_workflow)
        self.assertNotIn("branches:", self.release_workflow)
        self.assertIn("runs-on: ubuntu-24.04", self.release_workflow)
        self.assertIn("ref: ${{ github.sha }}", self.release_workflow)
        self.assertIn("fetch-depth: 0", self.release_workflow)
        self.assertIn('tag_ref="refs/tags/${GITHUB_REF_NAME}"', self.release_workflow)
        self.assertIn('test "${GITHUB_REF}" = "${tag_ref}"', self.release_workflow)
        self.assertIn('git show-ref --verify --quiet "${tag_ref}"', self.release_workflow)
        self.assertIn('git cat-file -t "${tag_ref}"', self.release_workflow)
        self.assertIn('git rev-parse HEAD)" = "${GITHUB_SHA}"', self.release_workflow)
        self.assertIn('git rev-parse "${tag_ref}^{}")" = "${GITHUB_SHA}"', self.release_workflow)
        self.assertIn(
            "python3 scripts/release/validate_version.py --tag \"${GITHUB_REF_NAME}\"",
            self.release_workflow,
        )

    def test_release_builds_both_packages_and_checks_contents(self) -> None:
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", self.release_workflow)
        self.assertIn("cpack --config build/CPackConfig.cmake -G TGZ -B dist", self.release_workflow)
        self.assertIn("cpack --config build/CPackConfig.cmake -G DEB -B dist", self.release_workflow)
        self.assertIn("tar -tzf", self.release_workflow)
        self.assertIn("dpkg-deb --contents", self.release_workflow)
        self.assertIn("Smoke-install DEB in Ubuntu 24.04 and run the demo", self.release_workflow)
        self.assertIn("docker run --rm --platform linux/amd64", self.release_workflow)
        self.assertIn("ubuntu:24.04", self.release_workflow)
        self.assertIn("apt-get install --yes --no-install-recommends", self.release_workflow)
        self.assertIn('test -s "${demo_dir}/demo_solution.pos"', self.release_workflow)
        self.assertIn('test -s "${demo_dir}/demo_solution.kml"', self.release_workflow)
        self.assertIn('test -s "${demo_dir}/demo_summary.json"', self.release_workflow)
        self.assertIn("SHA256SUMS", self.release_workflow)
        self.assertIn("sha256sum -c SHA256SUMS", self.release_workflow)
        self.assertIn("actions/upload-artifact@v7", self.release_workflow)
        self.assertIn("if-no-files-found: error", self.release_workflow)

    def test_release_creation_is_tag_verified_and_asset_upload_is_retry_safe(self) -> None:
        self.assertIn("gh release create", self.release_workflow)
        self.assertIn("--verify-tag", self.release_workflow)
        self.assertIn('gh release create "${RELEASE_TAG}" --verify-tag', self.release_workflow)
        self.assertIn("--generate-notes", self.release_workflow)
        self.assertIn('notes_file="docs/releases/${RELEASE_TAG}.md"', self.release_workflow)
        self.assertIn("--notes-file", self.release_workflow)
        self.assertIn("gh release upload", self.release_workflow)
        self.assertIn("--clobber", self.release_workflow)
        self.assertNotIn("git tag", self.release_workflow)

    def test_release_upload_is_limited_to_three_validated_regular_files(self) -> None:
        self.assertIn("id: package_assets", self.release_workflow)
        self.assertIn('test -f "${tgz[0]}"', self.release_workflow)
        self.assertIn('test ! -L "${tgz[0]}"', self.release_workflow)
        self.assertIn('test -f "${deb[0]}"', self.release_workflow)
        self.assertIn('test ! -L "${deb[0]}"', self.release_workflow)
        self.assertIn("test -f \"${checksums}\"", self.release_workflow)
        self.assertIn("test ! -L \"${checksums}\"", self.release_workflow)
        self.assertIn("printf 'tgz=%s\\n' \"${tgz[0]}\"", self.release_workflow)
        self.assertIn("printf 'deb=%s\\n' \"${deb[0]}\"", self.release_workflow)
        self.assertIn("printf 'checksums=%s\\n' \"${checksums}\"", self.release_workflow)
        self.assertIn("TGZ_ASSET: ${{ steps.package_assets.outputs.tgz }}", self.release_workflow)
        self.assertIn("DEB_ASSET: ${{ steps.package_assets.outputs.deb }}", self.release_workflow)
        self.assertIn("CHECKSUMS_ASSET: ${{ steps.package_assets.outputs.checksums }}", self.release_workflow)
        self.assertIn('"${TGZ_ASSET}"', self.release_workflow)
        self.assertIn('"${DEB_ASSET}"', self.release_workflow)
        self.assertIn('"${CHECKSUMS_ASSET}"', self.release_workflow)
        self.assertNotIn("gh release upload \"${RELEASE_TAG}\" dist/*", self.release_workflow)
        self.assertNotIn("gh release upload \"${RELEASE_TAG}\" dist/", self.release_workflow)

    def test_release_uses_only_the_contents_write_permission(self) -> None:
        self.assertIn("permissions:\n  contents: write", self.release_workflow)
        self.assertNotIn("packages:", self.release_workflow)
        self.assertNotIn("actions:", self.release_workflow)

    def test_debian_dependencies_cover_installed_surfaces(self) -> None:
        cmake_text = (ROOT_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("CPACK_DEBIAN_PACKAGE_DEPENDS", cmake_text)
        for dependency in ("libeigen3-dev", "python3", "python3-numpy", "python3-matplotlib"):
            with self.subTest(dependency=dependency):
                self.assertIn(dependency, cmake_text)
        self.assertIn("CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON", cmake_text)

    def test_docker_metadata_keeps_existing_tags_and_adds_minor_semver_aliases(self) -> None:
        tag_lines = {
            line.strip()
            for line in self.docker_workflow.splitlines()
            if line.strip().startswith("type=")
        }
        self.assertIn("type=ref,event=branch", tag_lines)
        self.assertIn("type=ref,event=tag", tag_lines)
        self.assertIn("type=semver,pattern={{version}}", tag_lines)
        self.assertIn("type=semver,pattern={{major}}.{{minor}}", tag_lines)
        self.assertIn("type=sha,format=short", tag_lines)
        self.assertIn("type=raw,value=latest,enable=${{ github.ref == 'refs/heads/main' }}", tag_lines)
        self.assertNotIn("type=semver,pattern={{major}}", tag_lines)

        compose_text = (ROOT_DIR / "compose.yaml").read_text(encoding="utf-8")
        self.assertIn(PUBLIC_IMAGE, compose_text)

    def test_public_onboarding_docs_use_stable_image(self) -> None:
        for relative_path in ("README.md", "docs/quickstart.md", "docs/self_contained_demo.md"):
            text = (ROOT_DIR / relative_path).read_text(encoding="utf-8")
            with self.subTest(path=relative_path):
                self.assertIn(PUBLIC_IMAGE, text)
                legacy_image = "ghcr.io/rsasaki0109/gnssplusplus-library:" + "develop"
                self.assertNotIn(legacy_image, text)


if __name__ == "__main__":
    unittest.main()
