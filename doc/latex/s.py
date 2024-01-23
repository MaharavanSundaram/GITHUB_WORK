import pypandoc
inputfile = "coinesAPI.tex"
output = pypandoc.convert_file(inputfile,'html',outputfile="s.html")
