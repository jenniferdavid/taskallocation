% Software License Agreement
%
% Author    Mike Purvis <mpurvis@clearpathrobotics.com>
% Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.
%
% Redistribution and use in source and binary forms, with or without modification, is
% not permitted without the express permission of Clearpath Robotics.

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{clearpathmanual}[2014/06/02 v1.0 Clearpath User Manual Class]

% Passes and class options to the underlying article class
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions

\LoadClass[11pt]{article}


%% Setup typefaces

\RequirePackage{microtype}
\RequirePackage{fontspec}
\RequirePackage{xltxtra,xunicode}
\RequirePackage{titlesec}
\RequirePackage{siunitx}
\defaultfontfeatures{Mapping=tex-text,Scale=MatchLowercase}

\setmainfont{Arial}
\setsansfont{Arial}
\setmonofont{Consolas}
\newfontfamily{\titlefont}{DINPro}

%% Define Clearpath colours

\RequirePackage{xcolor}
\definecolor{orange}{RGB}{243,130,55}
\definecolor{grey}{RGB}{71,71,71}
\definecolor{lightgrey}{RGB}{228,228,228}

%% Make TOC links black rather than blue
\AtBeginDocument{
\hypersetup{colorlinks=true, linkcolor=black}
}

%% Set up page formatting

\RequirePackage[letterpaper, total={6.0in, 8in}, top=1.75in, left=1.5in]{geometry}
\newcommand{\sectionbreak}{\clearpage\thispagestyle{plain}}
\pagenumbering{arabic}

%% Placeholder image for missing figures

\newcommand{\placeholder}[2]{\textcolor{lightgrey}{\rule[-2pt]{#1}{#2}}}

%% Set up heading

\RequirePackage{titlesec}
\titleformat{\section}
  {\titlefont\fontsize{24pt}{6pt}\bfseries\color{orange}}{\hspace*{-12.8pt}\llap\thesection\hskip 12pt}{0pt}{\MakeUppercase}
\titleformat{\subsection}
  {\titlefont\fontsize{18pt}{4pt}\bfseries\color{grey}}{\hspace*{-12.8pt}\llap\thesubsection\hskip 12pt}{0pt}{}

%% Set up source code listings

\RequirePackage{upquote}
\RequirePackage{listings}
\lstset{
  basicstyle=\ttfamily\footnotesize,
  breaklines=true,
  upquote=false,
  backgroundcolor=\color{lightgrey},
  aboveskip=8pt,
  belowskip=-1pt,
  framexleftmargin=5pt,
  framexrightmargin=5pt,
  framextopmargin=2pt,
  framexbottommargin=2pt,
  frame=tb, framerule=0pt
}

\RequirePackage{longtable}

\RequirePackage{tikz}
\usetikzlibrary{calc}
\RequirePackage{graphicx}
\RequirePackage[setpagesize=false, % page size defined by xetex
            unicode=false, % unicode breaks when used with xetex
            xetex]{hyperref}
\hypersetup{colorlinks=true,urlcolor=blue}

%% Compact list formatting option

\RequirePackage{enumitem}

%% Clearpath watermark in the top right corner of each page

\RequirePackage{background}
\backgroundsetup{
  scale=1,
  angle=0,
  opacity=1,
  color=black,
  contents={
    \begin{tikzpicture}[remember picture,overlay]
      \node at ([xshift=3.495in,yshift=3.6in] current page.north west)
      {\includegraphics[width=2.0in]{clearpath-latex/clearpath-logo.pdf}};
    \end{tikzpicture}
  }
}

\setlength{\parindent}{0pt}
\setlength{\parskip}{6pt plus 2pt minus 1pt}
\setlength{\emergencystretch}{3em}  % prevent overfull lines

\newcommand{\manualcover}[1]{
  \thispagestyle{empty}
  \begin{tikzpicture}[remember picture,overlay]
  \node[anchor=north west,inner sep=0pt] at ($(current page.north west)+(0cm,0cm)$) {
    \includegraphics{#1}
  };
  \end{tikzpicture}
}

%% Create macros for callout info boxes

\RequirePackage[many]{tcolorbox}

\newcommand{\MkAlertBox}[1]{
  \newtcolorbox{#1}[1][]{
    title=##1,
    colback=#1bg, colbacktitle=#1bg,
    coltitle=black,
    fonttitle=\bfseries,
    bottomrule=1pt, toprule=1pt,
    leftrule=1pt, rightrule=1pt,
    titlerule=0pt,
    outer arc=4pt,
    colframe=#1border,
    enlarge top initially by=5pt,
    theorem style=margin break, left=8pt, oversize
  }
}

\definecolor{noteborder}{RGB}{180,190,230}
\definecolor{notebg}{RGB}{220,232,255}
\MkAlertBox{note}

\definecolor{warningborder}{RGB}{220,150,150}
\definecolor{warningbg}{RGB}{255,190,190}
\MkAlertBox{warning}

\endinput
