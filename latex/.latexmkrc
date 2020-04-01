#!/usr/bin/env perl
##============================================================================##
## description  latexmk config file
## author       Ryotaro Onuki (GitHub: kerikun11)
## created_at   2019.01.25
## modified_at  2020.02.10
##============================================================================##
## latex commands; %S: src, %O: option, %D: dest, %B: src without extension
$pdflatex   = 'lualatex --synctex=1 --interaction=nonstopmode --halt-on-error --file-line-error %O %S';
$max_repeat = 4;
# $latex      = 'platex -synctex=1 -halt-on-error -interaction=nonstopmode %O %S';
# $bibtex     = 'pbibtex -kanji=utf8 %O %B';
# $biber      = 'biber --bblencoding=utf8 -u -U --output_safechars';
# $dvipdf     = 'dvipdfmx %O -o %D %S';
# $makeindex  = 'mendex %O -o %D %S';
# $bibtex_use = 2;
##============================================================================##
## pdf mode
$pdf_mode   = 1; # 0: none, 1: pdflatex, 2: ps2pdf, 3: dvipdf
##============================================================================##
## output directory
$aux_dir    = "build";
$out_dir    = "build";
##============================================================================##
## configure theme path
ensure_path('TEXINPUTS', './theme');
##============================================================================##
## avoid log linebreaks; see https://tex.stackexchange.com/questions/384133/
$ENV{max_print_line} = $log_wrap = 10000;
##============================================================================##
## viewer settings
$pvc_view_file_via_temporary = 0;
if ($^O eq 'darwin') {
    # OSX
    $pdf_previewer = 'open -a Preview';
} elsif ($^O eq 'linux') {
    # Linux
    $pdf_previewer = 'evince';
} else {
    # Windows
    $pdf_previewer = '"C:\Program Files\SumatraPDF\SumatraPDF.exe" -reuse-instance %O %S';
}
##============================================================================##
