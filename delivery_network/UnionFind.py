class UnionFind :
    """"""
    def __init__(self,x):
        self.element(x)
        self.parent=self

    def Find(self) :
        if self.parent == self :
            return self
        else :
            self.parent = self.parent.Find()
            return self.parent()

    def Union(x,y)


