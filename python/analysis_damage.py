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
from matplotlib import rcParams



def load_results(arg):

    if(len(arg)!=0):
        base = arg[0]
    else:
        base = "."

    files_2d = glob.glob(base+'/low_dim/results/*/*/results.dat')
    files_8d =  glob.glob(base+'/high_dim/results/*/*/results.dat')
    files_hbr =  glob.glob(base+'/hbr_simu/results/*/*/results.dat')
    files_aprol =  glob.glob(base+'/aprol/results/*/*/results.dat')
    
    files = []


    cols = ['found','actions','collisions','total_time']

    if files_2d:
        all_files = []
        for i,f in enumerate(files_2d):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['rep_nr'] = os.path.basename(os.path.dirname(os.path.dirname(f)))
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_2d = pd.concat(all_files, ignore_index=True)
        result_2d['variant'] = '2d'
        files.append(result_2d)
    if files_8d:
        all_files = []
        for i,f in enumerate(files_8d):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['rep_nr'] = os.path.basename(os.path.dirname(os.path.dirname(f)))
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_8d  =  pd.concat(all_files, ignore_index=True)
        result_8d['variant'] = '8d'
        files.append(result_8d)

    if files_hbr:
        all_files = []
        for i,f in enumerate(files_hbr):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['rep_nr'] = os.path.basename(os.path.dirname(os.path.dirname(f)))
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_hbr = pd.concat(all_files, ignore_index=True)
        result_hbr['variant'] = 'hbr'
        files.append(result_hbr)

    if files_aprol:
        all_files = []
        for i,f in enumerate(files_aprol):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,skiprows=1,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['rep_nr'] = os.path.basename(os.path.dirname(os.path.dirname(f)))
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_aprol = pd.concat(all_files, ignore_index=True)
        result_aprol['variant'] = 'aprol'
        files.append(result_aprol)
    
    
    
    data = pd.concat(files)
    data['rep_nr'] = pd.factorize(data['rep_nr'])[0]
    print(data)
    # data = data[data['found']==1]
    return data

def load_iters(arg):

    if(len(arg)!=0):
        base = arg[0]
    else:
        base = "."

    files_2d = glob.glob(base+'/low_dim/results/*/*/iter_1.dat')
    files_8d =  glob.glob(base+'/high_dim/results/*/*/iter_1.dat')
    files_hbr =  glob.glob(base+'/hbr_simu/results/*/*/iter_1.dat')
    files_aprol =  glob.glob(base+'/aprol/results/*/*/iter_1.dat')

    files = []

    cols = ['iter','time','random_actions_time','policy_calls','random_calls','tree_children','tree_depth']

    if files_2d:
        all_files = []
        for i,f in enumerate(files_2d):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_2d = pd.concat(all_files, ignore_index=True)
        result_2d['variant'] = '2d'
        files.append(result_2d)
    if files_8d:
        all_files = []
        for i,f in enumerate(files_8d):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_8d  =  pd.concat(all_files, ignore_index=True)
        result_8d['variant'] = '8d'
        files.append(result_8d)

    if files_hbr:
        all_files = []
        for i,f in enumerate(files_hbr):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_hbr = pd.concat(all_files, ignore_index=True)
        result_hbr['variant'] = 'hbr'
        files.append(result_hbr)

    if files_aprol:
        all_files = []
        for i,f in enumerate(files_aprol):
            t_df = pd.read_csv(f,delim_whitespace=True,names=cols,usecols=range(len(cols)),low_memory=False,dtype=np.float32)
            t_df['path'] = f
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_aprol = pd.concat(all_files, ignore_index=True)
        result_aprol['variant'] = 'aprol'
        files.append(result_aprol)


    data = pd.concat(files)

    return data

def load_dmg(arg):

    if(len(arg)!=0):
        base = arg[0]
    else:
        base = "."

    # files_2d = glob.glob(base+'/low_dim/results/*/*/dmg_1.dat')
    # files_8d =  glob.glob(base+'/high_dim/results/*/*/dmg_1.dat')
    files_hbr =  glob.glob(base+'/hbr_simu/results/*/*/dmg_1.dat')
    # files_aprol =  glob.glob(base+'/aprol/results/*/*/dmg_1.dat')

    files = []

    cols = ['']

    if files_hbr:
        all_files = []
        for f in files_hbr:
            t_df = pd.read_csv(f,delim_whitespace=True,low_memory=False,header=None,dtype=np.float32)
            t_df['path'] = f
            t_df['damage'] = int(os.path.basename(os.path.dirname(f)))
            all_files.append(t_df)
        result_hbr = pd.concat(all_files, ignore_index=True)
        result_hbr['variant'] = 'hbr'
        files.append(result_hbr)


    data = pd.concat(files)

    return data


def plot_steps(data,hue=False,name=''):
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

        pos = range(len(nobs))
        print(nobs.index)
        print(data)

        plt.figure()
        if not hue:
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
        else:
            sns_plot = sns.boxplot(x="variant", y=item,hue='damage',
                                    data=data,showmeans=True,order=medians.index)

        plt.savefig(item+"_"+name+".png")
        plt.close()
        plt.figure()

        sns_plot = sns.boxplot(x="damage", y=item,hue='variant',
                                    data=data,showmeans=True)

        plt.savefig(item+"_"+name+"_comp.png")
        plt.close()

def plot_steps_violin(data,name=''):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    for item in ["actions"]:

        print(data.groupby(['variant'])[item].median())
        print(data.groupby(['variant'])[item].mean())
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
        # sns_plot = sns.boxplot(x="variant", y=item,
        #                         data=data,showmeans=True,order=medians.index)
        sns_plot = sns.violinplot(x="variant", y=item, data=data, inner=None,palette="pastel")

        # sns_plot = sns.swarmplot(x="variant", y=item, data=data,color="white", edgecolor="gray")
        sns_plot = sns.swarmplot(x="variant", y=item, data=data,hue="found",palette='rocket')
        # sns_plot = sns.catplot(x="variant", y=item,
        #         hue="found",
        #         data=data, kind="swarm",
        #         height=4, aspect=.7);

        # for tick,label in zip(pos,medians.index):
        #     sns_plot.text(pos[tick],
        #         medians.loc[label][0] + 0.03,
        #         medians.loc[label][1],
        #         horizontalalignment='center',
        #         size='x-small',
        #         color='k',
        #         weight='semibold')

        plt.savefig(item+"_"+name+"_violin.png")
        plt.close()


def plot_valid_count(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    means = []
    for variant in data['variant'].unique():
        means.append([data[data['variant']==variant]["found"].sum()/data[data['variant']==variant]["found"].size,variant])

    for item in ["found"]:
        plt.figure()
        sns_plot = sns.countplot(x="variant",
                                data=data)
        plt.savefig(item+".png")
        plt.close()


    df = pd.DataFrame(means,columns=['percentage','variant'])
    plt.figure()
    ax = sns.barplot(x='variant', y="percentage", data=df)
    ax.set(ylabel="Percent")
   
    plt.savefig(item+"_percentage.png")
    plt.close()


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
    for item in ["time",'random_actions_time',"policy_calls","random_calls"]:
        plt.figure()
        sns_plot = sns.boxplot(x="found",y=item,hue="variant",
                                data=filtered_df,showmeans=True)
        plt.savefig(item+"_grouped.png")
        plt.close()

    for item in ["time",'random_actions_time']:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+".png")
        plt.close()

    for item in ["diff"]:
        plt.figure()
        sns_plot = sns.boxplot(y=item,
                                data=df,showmeans=True)
        plt.savefig(item+".png")
        plt.close()

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
        plt.close()


def plot_total_time(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data
    for item in ["total_time"]:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+".png")
        plt.close()

def plot_total_time_violin(data):
    sns.set(style="darkgrid")
    # Plot the responses for different events and regions
    filtered_df = data[data['variant']!= '8d']
    for item in ["total_time"]:
        plt.figure()
        sns_plot = sns.violinplot(x="variant",y=item,
                                data=filtered_df)
        plt.savefig(item+"_violin.png")
        plt.close()


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
        plt.close()

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
        plt.close()


def plot_random_search_time(data,step=0):
    sns.set(style="darkgrid")
    sns.set(rc={'figure.figsize':(20,10)})
    # Plot the responses for different events and regions
    filtered_df = data[data['iter']==step]
    filtered_df = filtered_df.groupby(["variant","path"],as_index=False)[['random_actions_time','time']].mean()
    print(filtered_df.groupby(["variant"],as_index=False).median())
    print(filtered_df.groupby(["variant"],as_index=False).mean())


    for item in ['time','random_actions_time']:
        plt.figure()
        sns_plot = sns.boxplot(x="variant",y=item,
                                data=filtered_df,showmeans=True)
        plt.savefig(item+'_'+str(step)+".png")
        plt.close()

def matching(x):
    real_damage = x[[1,2,3,4,5,6]].values
    predicted_damage = x[[7,8,9,10,11,12]].values
    index_damage = np.where(real_damage == 0)[0][0]
    if (predicted_damage==real_damage).all():
        return 2
    elif predicted_damage[index_damage]==0:
        return 1
    else:
        return 0

def plot_action_choice(data):
    sns.set(style="darkgrid")
    data['Match'] = data.apply(lambda x: matching(x),axis=1)
    data['Simple_Match'] = data['Match'].apply(lambda x: 1 if(x>0) else 0)

    sns.set(rc={'figure.figsize':(20,10)})
    
    print(data)
    filtered_df = data.groupby(["variant","damage"],as_index=False)[['Simple_Match']].mean()
    print(filtered_df)
    percentages = (data.groupby(["variant","damage"])['Match']
                        .value_counts(normalize=True)
                        .mul(100)
                        .rename('percent')
                        .reset_index())

    for item in ['Match']:
        plt.figure()
        sns_plot = sns.barplot(data=percentages, x='damage',y='percent',hue=item)
        plt.savefig(item+"_Percentages.png")

        plt.close()

    for item in ['Simple_Match']:
        plt.figure()
        sns_plot = sns.barplot(x="variant", y=item,hue='damage',
                                    data=filtered_df)

        plt.savefig(item+"_Acc.png")
        plt.close()


def plot_steps_per_repertoire(data,name=''):
    sns.set(style="darkgrid")
    plt.rcParams["figure.figsize"] = [10,10]
    # Plot the responses for different events and regions
    for item in ["actions"]:

        medians = data.groupby(['variant'])[item].median()#.values

        test = data.groupby(['variant','rep_nr'],as_index=False)[item].mean()
        print(test)
        test = test.groupby(['variant'])[item].median()
        print(test)

        print(test)
        nobs = data["variant"].value_counts()
        nobs = nobs.apply(lambda x: "n: "+str(x))
        medians = pd.concat([medians,nobs],axis=1)
        medians.columns = ['median',"count"]
        print(data)

        pos = range(len(nobs))
        plt.figure()

        sns_plot = sns.boxplot(x="rep_nr", y=item,hue='variant',
                                    data=data,showmeans=True,palette='pastel')
        sns_plot = sns.swarmplot(x="rep_nr", y=item, data=data,hue='damage',palette='rocket')

        plt.savefig(item+"_"+name+"_per_repertoire.png")
        plt.close()

        test = data.groupby(['variant','rep_nr'],as_index=False)[item].median()
        plt.figure()

        sns_plot = sns.violinplot(x="variant", y=item, data=test,showmeans=True,palette='pastel',inner = 'quartile',cut=0)

        sns_plot = sns.swarmplot(x="variant", y=item, data=test,palette='rocket',color="white", edgecolor="gray")

        plt.savefig(item+"_"+name+"_per_repertoire_median.png")
        plt.close()

        test = data.groupby(['variant','rep_nr'],as_index=False)[item].quantile(.75)
        plt.figure()
        sns_plot = sns.violinplot(x="variant", y=item, data=test,showmeans=True,palette='pastel',inner = 'quartile',cut=0)

        sns_plot = sns.swarmplot(x="variant", y=item, data=test,palette='rocket',color="white", edgecolor="gray")


        plt.savefig(item+"_"+name+"_per_repertoire_75per.png")
        plt.close()

        test = data.groupby(['variant','rep_nr'],as_index=False)[item].quantile(.25)
        plt.figure()

        sns_plot = sns.violinplot(x="variant", y=item, data=test,showmeans=True,palette='pastel',inner = 'quartile',cut=0)

        sns_plot = sns.swarmplot(x="variant", y=item, data=test,palette='rocket',color="white", edgecolor="gray")


        plt.savefig(item+"_"+name+"_per_repertoire_25per.png")
        plt.close()

        test = data.groupby(['variant','rep_nr'],as_index=False)[item].quantile(.75,interpolation='lower')
        plt.figure()

        sns_plot = sns.violinplot(x="variant", y=item, data=test,showmeans=True,palette='pastel',inner = 'quartile',cut=0)

        sns_plot = sns.swarmplot(x="variant", y=item, data=test,palette='rocket',color="white", edgecolor="gray")


        plt.savefig(item+"_"+name+"_per_repertoire_75per_point.png")
        plt.close()

        test = data.groupby(['variant','rep_nr'],as_index=False)[item].quantile(.25,interpolation='lower')
        plt.figure()

        sns_plot = sns.violinplot(x="variant", y=item, data=test,showmeans=True,palette='pastel',inner = 'quartile',cut=0)

        sns_plot = sns.swarmplot(x="variant", y=item, data=test,palette='rocket',color="white", edgecolor="gray")


        plt.savefig(item+"_"+name+"_per_repertoire_25per_point.png")
        plt.close()





if __name__ == "__main__":
    print('Collecting Progress')
    import os

    # define the name of the directory to be created
    path = "./UPLOAD"
    os.makedirs(path,exist_ok=True)
    results = load_results(sys.argv[1:])

    plot_valid_count(results)
    # results = results[results['damage']!=6]

    plot_steps(results,True,name = 'all')
    plot_steps_violin(results,name = 'all')
    plot_steps_per_repertoire(results,name='all')


    results = results[results['found']==1]


    plot_steps_per_repertoire(results,name='')
    plot_steps(results,True)
    plot_steps_violin(results)
    


    iterations = load_iters(sys.argv[1:])
    dmg = load_dmg(sys.argv[1:])
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
    plot_random_search_time(iterations)
    plot_action_choice(dmg)

