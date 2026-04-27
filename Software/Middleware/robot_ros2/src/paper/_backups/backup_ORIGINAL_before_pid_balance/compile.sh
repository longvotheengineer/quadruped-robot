#!/bin/bash
# Simple script to compile the LaTeX paper

echo "Compiling LaTeX document..."
pdflatex -interaction=nonstopmode main.tex
bibtex main
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex
echo "Done! Check main.pdf"
