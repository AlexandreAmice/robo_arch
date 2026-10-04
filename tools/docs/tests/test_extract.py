"""Exercise Clang comment ownership, rendering and failure diagnostics."""

import pytest

from tools.docs.extract import extract, render


def declaration(name="f", text="Shared description.", signature="double (double)"):
    return {
        "kind": "FunctionDecl",
        "name": name,
        "type": {"qualType": signature},
        "inner": [
            {
                "kind": "FullComment",
                "inner": [
                    {
                        "kind": "ParagraphComment",
                        "inner": [{"kind": "TextComment", "text": text}],
                    }
                ],
            }
        ],
    }


def test_select_overload_and_propagate_comment():
    manifest = {"F": {"symbol": "example::f", "signature": "double (double)"}}
    selected = declaration()
    ast = {
        "kind": "NamespaceDecl",
        "name": "example",
        "inner": [
            selected,
            declaration(text="Integer overload.", signature="int (int)"),
        ],
    }
    assert extract(ast, manifest) == {"F": "Shared description."}
    selected["inner"][0]["inner"][0]["inner"][0]["text"] = "Updated description."
    assert extract(ast, manifest) == {"F": "Updated description."}


def test_missing_or_duplicate_owner_fails():
    manifest = {"F": {"symbol": "f", "signature": "double (double)"}}
    with pytest.raises(ValueError, match="Expected one"):
        extract({"kind": "TranslationUnitDecl"}, manifest)
    with pytest.raises(ValueError, match="Expected one"):
        extract({"inner": [declaration(), declaration()]}, manifest)
    with pytest.raises(ValueError, match="Duplicate documentation selection"):
        extract(declaration(), {"F": manifest["F"], "ALIAS": manifest["F"]})
    with pytest.raises(ValueError, match="Missing documentation"):
        extract({**declaration(), "inner": []}, manifest)


def test_unsupported_markup_and_unknown_parameters_fail():
    with pytest.raises(ValueError, match="Unsupported Doxygen"):
        render({"kind": "HTMLStartTagComment", "name": "table"})
    with pytest.raises(ValueError, match="Unknown parameter"):
        render({"kind": "ParamCommandComment", "param": "typo"})


def test_multiline_rst_preserves_lists_and_relative_indentation():
    # These paragraph/text nodes match Clang's AST for consecutive /// lines.
    paragraph = {
        "kind": "ParagraphComment",
        "inner": [
            {"kind": "TextComment", "text": " - first value"},
            {"kind": "TextComment", "text": "   continuation"},
            {"kind": "TextComment", "text": " - second value"},
        ],
    }
    assert render(paragraph) == "- first value\n  continuation\n- second value"
    note = {"kind": "BlockCommandComment", "name": "note", "inner": [paragraph]}
    assert render(note) == (
        ".. note::\n\n    - first value\n      continuation\n    - second value"
    )
    param = {
        "kind": "ParamCommandComment",
        "param": "value",
        "paramIdx": 0,
        "inner": [paragraph],
    }
    assert render(param) == (
        ":param value: - first value\n      continuation\n    - second value"
    )
    literal = {
        "kind": "ParagraphComment",
        "inner": [
            {"kind": "TextComment", "text": "     first_value()"},
            {"kind": "TextComment", "text": "     second_value()"},
        ],
    }
    assert render(literal) == "    first_value()\n    second_value()"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
