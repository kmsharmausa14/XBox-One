namespace FingerTracking
{
    partial class MainWindow
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.trackingImage = new System.Windows.Forms.PictureBox();
            ((System.ComponentModel.ISupportInitialize)(this.trackingImage)).BeginInit();
            this.SuspendLayout();
            // 
            // trackingImage
            // 
            this.trackingImage.BackColor = System.Drawing.SystemColors.InactiveCaption;
            this.trackingImage.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.trackingImage.Location = new System.Drawing.Point(0, 0);
            this.trackingImage.Name = "trackingImage";
            this.trackingImage.Size = new System.Drawing.Size(800, 600);
            this.trackingImage.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.trackingImage.TabIndex = 0;
            this.trackingImage.TabStop = false;
            // 
            // MainWindow
            // 

            
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(800, 600);
            this.Controls.Add(this.trackingImage);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.SizableToolWindow;
            this.Name = "MainWindow";
            this.Text = "Fingertracking Test Panel";
            ((System.ComponentModel.ISupportInitialize)(this.trackingImage)).EndInit();
            this.ResumeLayout(false);
            
        }

        #endregion

   
        private System.Windows.Forms.PictureBox trackingImage;
      
    }
}

