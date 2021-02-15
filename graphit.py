import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

data = pd.read_csv('data.csv')
# Data columns:
# Freq, Real, Img
palette = sns.dark_palette("#69d", reverse=True, as_cmap=True)   # Blues

# Calculate the magnitude
data['mag'] = pow(pow(data['Real'],2)+pow(data['Img'],2), 0.5)

ax = sns.lineplot(x="Freq", y="mag", 
  palette=palette,
  data=data)
plt.xlim(None, None)
plt.xlabel("Frequency")
plt.ylabel('Magnitude')
plt.show()   

ax = sns.lineplot(x="Real", y="Img", 
  palette=palette,
  data=data)
plt.xlim(None, None)
plt.xlabel("Real")
plt.ylabel('Imag')
plt.show()   

