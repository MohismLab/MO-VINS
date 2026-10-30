#!/bin/sh
# Run inside a XeLaTeX environment with the Noto CJK fonts installed.
set -eu
cd "$(dirname "$0")"
mkdir -p ../../out/latex ../pdf
for name in specification user-manual; do
 xelatex -no-pdf -interaction=nonstopmode -halt-on-error -output-directory=../../out/latex "$name.tex" > "../../out/latex/$name-build.log" 2>&1
 xelatex -no-pdf -interaction=nonstopmode -halt-on-error -output-directory=../../out/latex "$name.tex" >> "../../out/latex/$name-build.log" 2>&1
 xdvipdfmx -q -o "../../out/latex/$name.pdf" "../../out/latex/$name.xdv" >> "../../out/latex/$name-build.log" 2>&1
 cp "../../out/latex/$name.pdf" "../pdf/MOHISM-VINS-$name.pdf"
done
