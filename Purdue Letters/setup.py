

l1 = 8 # Length
l2 = 3.2 # Width
dz = .5

z1 = 1.2    # Height of stand
z2 = .2     # Height of small stand

anchorx = []
anchory = []
anchorz = []


seqx = [1, -1, -1, 1]
seqy = [-1, -1, 1, 1]
seqz = [0, 1, 1, 0]

for n in range(len(seqx)):
    anchorx.append(l2/2*seqx[n])
    anchory.append(l1/2*seqy[n])
    anchorz.append(z1 + dz*seqz[n])


seqx = [1, -1, -1, 1]
seqy = [-1, -1, 1, 1]
seqz = [0, 0, 1, 1]
for i in range(len(seqx)):
    anchorx.append(l2/4*seqx[i])
    anchory.append(l1/4*seqy[i])
    anchorz.append(z2 + dz*seqz[i])


seqx = [1, -1]
seqy = [0, 0]
seqz = [1, 0]
for n in range(len(seqx)):
    anchorx.append(l2/4*seqx[n])
    anchory.append(l1/4*seqy[n])
    anchorz.append(z1 + dz*seqz[n])

seqx = [0, 0]
seqy = [-1, 1]
seqz = [0, 0]
for n in range(len(seqx)):
    anchorx.append(l2/4*seqx[n])
    anchory.append(l1/4*seqy[n])
    anchorz.append(z2 + dz*seqz[n])


for n in range(len(anchorx)):
    print("Anchor {:>3} is at: {:>4},  {:>4},  {:>4}".format(n, anchorx[n],anchory[n],anchorz[n]))
