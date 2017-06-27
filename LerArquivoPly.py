from libs.geometry import Point

class PLY(object):

    def __init__(self, nome_arquivo ="/"):
        self.formato = ""
        self.autor = ""
        self.comentario = ""
        self.variaveis = []
        self.variavel = 0
        self.vertice = 0
        self.face  = 0

        self.vertex = []
        self.faces  = []

        end_header = 10000

        with open(nome_arquivo) as f:
            for ln, line in enumerate(f, 1):
                words = line.split()

                if not( end_header <= ln):
                    if ( words[0] == "ply"):
                        print " "
                    elif (words[0] == "format"):
                        self.formato = line
                    elif (words[0] == "comment"):
                        if (words[1] == "made"):
                            self.autor = " ".join(words[3:])
                        else:
                            self.comentario = " ".join(words[1:])
                    elif (words[0] == "element"):
                        if( words[1] == "face"):
                            self.face = int(words[2])
                        elif(words[1] == "vertex"):
                            self.vertice = int(words[2])
                    elif (words[0] == "property"):
                        if (words[1] == "float32"):
                            self.variavel += 1
                            self.variaveis.append(words[2])
                    elif (words[0] == "end_header"):
                        end_header = ln
                else:
                    if (end_header+self.vertice >= ln):
                        self.vertex.append(Point(float(words[0]),float(words[1]),float(words[2])))
                    else:
                        aux_points = []
                        for i in xrange(1,int(words[0])+1):
                            aux_points.append(int(words[i]))
                        self.faces.append(aux_points)
