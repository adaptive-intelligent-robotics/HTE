import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import subprocess
import os
import sys
import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import seaborn as sns



def load_results(arg):

    if(len(arg)!=0):
        base = arg[0]
    else:
        base = "."

    files_2d = glob.glob(base+'/low_dim/results/*/results.dat')
    files_8d =  glob.glob(base+'/high_dim/results/*/results.dat')
    files_hbr =  glob.glob(base+'/hbr_simu/results/*/results.dat')
    files_aprol =  glob.glob(base+'/aprol/results/*/results.dat')
    
    files = []


    cols = ['found','actions','collisions','total_time']

    if files_2d:
        all_files = []
        for f in files_2d:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_2d = pd.concat(all_files, ignore_index=True)
        result_2d['variant'] = '2d'
        files.append(result_2d)
    if files_8d:
        all_files = []
        for f in files_8d:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_8d  =  pd.concat(all_files, ignore_index=True)
        result_8d['variant'] = '8d'
        files.append(result_8d)

    if files_hbr:
        all_files = []
        for f in files_hbr:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_hbr = pd.concat(all_files, ignore_index=True)
        result_hbr['variant'] = 'hbr'
        files.append(result_hbr)

    if files_aprol:
        all_files = []
        for f in files_aprol:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_aprol = pd.concat(all_files, ignore_index=True)
        result_aprol['variant'] = 'aprol'
        files.append(result_aprol)
    
    
    
    data = pd.concat(files)
    print(data)
    # data = data[data['found']==1]
    return data

def load_iters(arg):

    if(len(arg)!=0):
        base = arg[0]
    else:
        base = "."

    files_2d = glob.glob(base+'/low_dim/results/*/iter_1.dat')
    files_8d =  glob.glob(base+'/high_dim/results/*/iter_1.dat')
    files_hbr =  glob.glob(base+'/hbr_simu/results/*/iter_1.dat')
    files_aprol =  glob.glob(base+'/aprol/results/*/iter_1.dat')
    files = []

    cols = ['iter','time','random_actions_time','policy_calls','random_calls','tree_children','tree_depth']
    
    if files_2d:
        all_files = []
        for f in files_2d:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_2d = pd.concat(all_files, ignore_index=True)
        result_2d['variant'] = '2d'
        files.append(result_2d)
    if files_8d:
        all_files = []
        for f in files_8d:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_8d  =  pd.concat(all_files, ignore_index=True)
        result_8d['variant'] = '8d'
        files.append(result_8d)

    if files_hbr:
        all_files = []
        for f in files_hbr:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_hbr = pd.concat(all_files, ignore_index=True)
        result_hbr['variant'] = 'hbr'
        files.append(result_hbr)

    if files_aprol:
        all_files = []
        for f in files_aprol:
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            all_files.append(t_df)
        result_aprol = pd.concat(all_files, ignore_index=True)
        result_aprol['variant'] = 'aprol'
        files.append(result_aprol)


    data = pd.concat(files)

    return data

def plot_steps(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    for item in ["actions"]:

        print(data.groupby(['variant'])[item].median())
        medians = data.groupby(['variant'])[item].median()#.values
        nobs = data["variant"].value_counts()
        print(data["variant"].value_counts())
        nobs = nobs.apply(lambda x: "n: "+str(x))
        medians = pd.concat([medians,nobs],axis=1)
        medians.columns = ['median',"count"]
        print(medians)
        # nobs = [str(x) for x in nobs.tolist()]
        # nobs = ["n: " + i for i in nobs]

        pos = range(len(nobs))
        print(nobs.index)

        plt.figure()
        sns_plot = sns.boxplot(x="variant", y=item,
                                data=data,showmeans=True,order=medians.index)

        for tick,label in zip(pos,medians.index):
            sns_plot.text(pos[tick],
                medians.loc[label][0] + 0.03,
                medians.loc[label][1],
                horizontalalignment='center',
                size='x-small',
                color='k',
                weight='semibold')

        plt.savefig(item+".png")

def plot_steps_violin(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    for item in ["actions"]:

        print(data.groupby(['variant'])[item].median())
        medians = data.groupby(['variant'])[item].median()#.values
        nobs = data["variant"].value_counts()
        print(data["variant"].value_counts())
        nobs = nobs.apply(lambda x: "n: "+str(x))
        medians = pd.concat([medians,nobs],axis=1)
        medians.columns = ['median',"count"]
        print(medians)
        # nobs = [str(x) for x in nobs.tolist()]
        # nobs = ["n: " + i for i in nobs]

        pos = range(len(nobs))
        print(nobs.index)
        plt.figure()
        sns_plot = sns.violinplot(x="variant", y=item,
                                data=data,showmeans=True,order=medians.index)

        for tick,label in zip(pos,medians.index):
            sns_plot.text(pos[tick],
                medians.loc[label][0] + 0.03,
                medians.loc[label][1],
                horizontalalignment='center',
                size='x-small',
                color='k',
                weight='semibold')

        plt.savefig(item+"_violin.png")


def plot_valid_count(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    means = []
    for variant in data['variant'].unique():
        means.append([data[data['variant']==variant]["found"].sum()/data[data['variant']==variant]["found"].size,variant])



    # hbr_mean = data[data['variant']=="hbr"]["found"].sum()/data[data['variant']=="hbr"]["found"].size
    # mean_2d = data[data['variant']=="2d"]["found"].sum()/data[data['variant']=="2d"]["found"].size
    # mean_aprol = data[data['variant']=="aprol"]["found"].sum()/data[data['variant']=="aprol"]["found"].size

    for item in ["found"]:
        plt.figure()
        sns_plot = sns.countplot(x="variant",
                                data=data)
        plt.savefig(item+".png")


    # df = pd.DataFrame([[mean_2d,'2d'],[hbr_mean,'hbr'],[mean_aprol,'aprol']],columns=['percentage','variant'])
    df = pd.DataFrame(means,columns=['percentage','variant'])
    plt.figure()
    ax = sns.barplot(x='variant', y="percentage", data=df)
    ax.set(ylabel="Percent")
    # sns_plot = sns.countplot(x="variant",
    #                         data=data)
    plt.savefig(item+"_percentage.png")


def plot_planning(data):
    sns.set(style="darkgrid")
    sns.set(rc={'figure.figsize':(20,10)})
    # Plot the responses for different events and regions
    filtered_df = data
    filtered_df = filtered_df.groupby(["variant","path"],as_index=False).sum()
    df = filtered_df.groupby(["variant","found"],as_index=False)['time'].mean()
    df['diff'] = df[["found","time"]].groupby("found",as_index=False).diff()
    print(df)
    print(filtered_df.groupby(["variant","path"],as_index=False).mean())
    for item in ["time","policy_calls","random_calls"]:
        plt.figure()
        sns_plot = sns.boxplot(x="found",y=item,hue="variant",
                                data=filtered_df,showmeans=True)
        plt.savefig(item+"_grouped.png")

    for item in ["time"]:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+".png")

    for item in ["diff"]:
        plt.figure()
        sns_plot = sns.boxplot(y=item,
                                data=df,showmeans=True)
        plt.savefig(item+".png")

def plot_planning_violin(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data[data['variant']!= '8d']
    df = filtered_df.groupby(["variant","path"],as_index=False).sum()
    for item in ["time"]:
        plt.figure()
        sns_plot = sns.violinplot(x="variant",y=item,
                                data=df)
        plt.savefig(item+"_violin.png")


def plot_total_time(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data
    for item in ["total_time"]:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+".png")

def plot_total_time_violin(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data[data['variant']!= '8d']
    for item in ["total_time"]:
        plt.figure()
        sns_plot = sns.violinplot(x="variant",y=item,
                                data=filtered_df)
        plt.savefig(item+"_violin.png")


def plot_policy_mean(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data
    filtered_df = filtered_df.groupby(["variant","path"],as_index=False).mean()
    for item in ["policy_calls",'random_calls']:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+"_mean.png")

def plot_policy_sum(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data
    filtered_df = filtered_df.groupby(["variant","path"],as_index=False).sum()
    for item in ["policy_calls",'random_calls']:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+"_sum.png")




if __name__ == "__main__":
    print('Collecting Progress')
    import os

    # define the name of the directory to be created
    path = "./UPLOAD"
    os.makedirs(path,exist_ok=True)
    results = load_results(sys.argv[1:])

    plot_valid_count(results)

    results = results[results['found']==1]

    plot_steps(results)
    plot_steps_violin(results)
    


    iterations = load_iters(sys.argv[1:])
    # print(iterations)
    results['path'] = results['path'].apply(lambda x: os.path.dirname(x))
    iterations['path'] = iterations['path'].apply(lambda x: os.path.dirname(x))
    iterations = results.merge(iterations, left_on=['path','variant'], right_on=['path','variant'])
    # print(iterations)
    plot_planning(iterations)
    plot_planning_violin(iterations)
    plot_total_time(results)
    plot_total_time_violin(results)
    plot_policy_mean(iterations)
    plot_policy_sum(iterations)

