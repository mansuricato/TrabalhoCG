try:
	from Tkinter import *
except ImportError as err:
	from tkinter import *
import sys
import os
class menu:
    def __init__(self, master):
        self.master = master
        master.title("CG 2017.1")

        #altera o tamanho do menu
        master.geometry("450x350")

        self.label = Label(master, text="Selecione a textura e o poligono", font=("Fixedsys",16))
        self.label.pack()

        #frame para os poligonos
        self.poligonos = Frame(master)
        self.poligonos.pack(side=RIGHT)

        #botao que fecha o menu
        self.close_button = Button(master, text="Fechar", command=master.quit)
        self.close_button.pack(in_=self.poligonos,side=BOTTOM,expand = True)
        self.close_button.config(width = 5)

        #variavel que salva a textura escolhida
        self.textura = StringVar(master)

        #botao para a textura 1
        self.imagem1 = PhotoImage(file='dragaoBranco.gif')
        self.botao_textura1 = Radiobutton(text='Dragão Branco',value='dragaoBranco.jpg',variable=self.textura)
        self.botao_textura1.pack(anchor=W)
        self.botao_textura1.config(compound=BOTTOM,image=self.imagem1)

        #botao para a textura 2
        self.imagem2 = PhotoImage(file='squirtle.gif')
        self.botao_textura2 = Radiobutton(text='Squirtle',value='squirtle.jpg',variable=self.textura)
        self.botao_textura2.pack(anchor=W)
        self.botao_textura2.config(compound=BOTTOM,image=self.imagem2)

        #botao para a textura 3
        self.imagem3 = PhotoImage(file='tentomon.gif')
        self.botao_textura3 = Radiobutton(text='Tentomon',value='tentomon.jpg',variable=self.textura)
        self.botao_textura3.pack(anchor=W)
        self.botao_textura3.config(compound=BOTTOM,image=self.imagem3)

        #botao que seleciona o cubo
        self.botao_cubo = Button(master, text="Cubo", command=self.Cubo)
        self.botao_cubo.pack(in_=self.poligonos,expand = True)
        self.botao_cubo.config(height=3, width=10)

        #botao que seleciona o tetraedro
        self.botao_tetraedro = Button(master, text="Tetraedro", command=self.Tetraedro)
        self.botao_tetraedro.pack(in_=self.poligonos,expand = True)
        self.botao_tetraedro.config(height=3, width=10)

        #botao que seleciona o octaedro
        self.botao_octaedro = Button(master, text="Octaedro", command=self.Octaedro)
        self.botao_octaedro.pack(in_=self.poligonos,expand = True)
        self.botao_octaedro.config(height=3, width=10)

        #botao que seleciona o dodecaedro
        self.botao_dodecaedro = Button(master, text="Dodecaedro", command=self.Dodecaedro)
        self.botao_dodecaedro.pack(in_=self.poligonos,expand = True)
        self.botao_dodecaedro.config(height=3, width=10)

        #botao que seleciona o icosaedro
        self.botao_icosaedro = Button(master, text="Icosaedro", command=self.Icosaedro)
        self.botao_icosaedro.pack(in_=self.poligonos,expand = True)
        self.botao_icosaedro.config(height=3, width=10)

    def Cubo(self):
        os.system('python Trabalho.py cube.ply '+self.textura.get())

    def Tetraedro(self):
        os.system('python Trabalho.py tetrahedron.ply '+self.textura.get())

    def Octaedro(self):
        os.system('python Trabalho.py octahedron.ply '+self.textura.get())

    def Dodecaedro(self):
        os.system('python Trabalho.py dodecahedron.ply '+self.textura.get())

    def Icosaedro(self):
        os.system('python Trabalho.py icosahedron.ply '+self.textura.get())

root = Tk()
my_gui = menu(root)
root.mainloop()
